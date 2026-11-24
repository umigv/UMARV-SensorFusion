#include "kalman_interface.h"
#include "kalman_internal.h"
#include "kalman_math.h"
#include "kalman_workspace.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const double pi=3.14159265358979323846;
static SrukfWorkspace work;
static void near(double a,double b,double tol) {
    if (!(fabs(a-b)<=tol)) {
        fprintf(stderr,"%.17g != %.17g (tol %.3g)\n",a,b,tol);assert(0);
    }
}
static Srukf initial(const double x[5],double variance) {
    Srukf f;SrukfConfig c;srukf_config_default(&c);
    c.vn_gate_threshold=c.encoder_gate_threshold=0;
    c.linear_accel_noise_density=c.angular_accel_noise_density=0;
    double P[5][5]={{0}};for(size_t i=0;i<5;++i)P[i][i]=variance;
    assert(srukf_init(&f,&c,0,x,P)==SRUKF_OK);return f;
}
static SrukfVnMeasurement vn(double t,const double x[5],double variance) {
    SrukfVnMeasurement m={0};m.timestamp=t;m.x=x[0];m.y=x[1];
    m.heading=x[2];m.angular_velocity=x[3];
    for(size_t i=0;i<4;++i)m.covariance[i][i]=variance;
    return m;
}
static SrukfEstimate estimate(const Srukf *f) {
    SrukfEstimate e;assert(srukf_get_estimate(f,&e)==SRUKF_OK);
    double P[25],L[25];
    for(size_t i=0;i<5;++i)for(size_t j=0;j<5;++j) {
        assert(isfinite(e.covariance[i][j]));near(e.covariance[i][j],e.covariance[j][i],1e-12);
        P[i*5+j]=e.covariance[i][j];
    }
    assert(srmat_cholesky_lower(P,5,5,L,5)==SRUKF_OK);
    assert(e.heading>=-pi && e.heading<pi);return e;
}

static void test_motion_and_noise(void) {
    double x[5]={2,-1,.4,.6,3},out[5];
    srukf_motion_model(x,.3,out);
    near(out[0],2+.9*cos(.49),1e-14);near(out[1],-1+.9*sin(.49),1e-14);
    near(out[2],.58,1e-14);near(out[3],.6,0);near(out[4],3,0);
    x[4]=-3;srukf_motion_model(x,.3,out);near(out[0],2-.9*cos(.49),1e-14);
    x[4]=0;srukf_motion_model(x,.3,out);near(out[0],2,0);near(out[2],.58,1e-14);
    x[2]=pi-.01;x[3]=1;srukf_motion_model(x,.02,x);near(x[2],-pi+.01,1e-14);
    SrukfConfig c;srukf_config_default(&c);double L[5][5],Q[5][5]={{0}};
    const double dt=.3;
    assert(srukf_process_noise_factor(&c,x,dt,L)==SRUKF_OK);
    for(size_t i=0;i<5;++i)for(size_t j=0;j<5;++j)
        for(size_t k=0;k<5;++k)Q[i][j]+=L[i][k]*L[j][k];
    double angle=x[2]+x[3]*dt/2,qa=c.linear_accel_noise_density,qw=c.angular_accel_noise_density;
    near(Q[0][0],qa*dt*dt*dt/3*cos(angle)*cos(angle),1e-14);
    near(Q[0][1],qa*dt*dt*dt/3*cos(angle)*sin(angle),1e-14);
    near(Q[0][4],qa*dt*dt/2*cos(angle),1e-14);
    near(Q[1][4],qa*dt*dt/2*sin(angle),1e-14);
    near(Q[4][4],qa*dt,1e-14);
    near(Q[2][2],qw*dt*dt*dt/3,1e-14);near(Q[2][3],qw*dt*dt/2,1e-14);
    near(Q[3][3],qw*dt,1e-14);
    assert(srukf_process_noise_factor(&c,x,0,L)==SRUKF_OK);
    for(size_t i=0;i<5;++i)for(size_t j=0;j<5;++j)near(L[i][j],0,0);
}

/* Gauss-Jordan test reference, independent of the library's triangular solves. */
static void inverse(const double A[4][4],size_t n,double inv[4][4]) {
    double B[4][8]={{0}};
    for(size_t i=0;i<n;++i){for(size_t j=0;j<n;++j)B[i][j]=A[i][j];B[i][n+i]=1;}
    for(size_t k=0;k<n;++k) {
        size_t pivot=k;for(size_t i=k+1;i<n;++i)if(fabs(B[i][k])>fabs(B[pivot][k]))pivot=i;
        for(size_t j=0;j<2*n;++j){double tmp=B[k][j];B[k][j]=B[pivot][j];B[pivot][j]=tmp;}
        double d=B[k][k];assert(fabs(d)>1e-14);
        for(size_t j=0;j<2*n;++j)B[k][j]/=d;
        for(size_t i=0;i<n;++i)if(i!=k){double m=B[i][k];for(size_t j=0;j<2*n;++j)B[i][j]-=m*B[k][j];}
    }
    for(size_t i=0;i<n;++i)for(size_t j=0;j<n;++j)inv[i][j]=B[i][n+j];
}
static void test_dense_update(SrukfSensor sensor,bool joint) {
    double x[5]={1,-2,.4,.1,2};
    /* Correlated SPD prior, including velocity/position/heading correlations. */
    const double L[5][5]={{.7,0,0,0,0},{.1,.6,0,0,0},{.02,-.03,.15,0,0},
                          {.01,.02,.01,.13,0},{.15,-.1,.04,.02,.4}};
    double P[5][5]={{0}};
    for(size_t i=0;i<5;++i)for(size_t j=0;j<5;++j)for(size_t k=0;k<5;++k)P[i][j]+=L[i][k]*L[j][k];
    SrukfConfig c;srukf_config_default(&c);c.correct_unmeasured_states=joint;
    c.vn_gate_threshold=c.encoder_gate_threshold=0;
    Srukf f;assert(srukf_init(&f,&c,0,x,P)==SRUKF_OK);
    double z[4]={1.2,-1.7,.5,.08},R[4][4]={{.1,.02,0,0},{.02,.1,0,0},{0,0,.02,.004},{0,0,.004,.03}};
    size_t m=sensor==SRUKF_SENSOR_VN?4:1, obs[4]={0,1,2,3};
    if(m==1){obs[0]=4;z[0]=2.4;R[0][0]=.07;}
    double C[4][4]={{0}},inv[4][4]={{0}},K[5][4]={{0}};
    for(size_t i=0;i<m;++i)for(size_t j=0;j<m;++j)C[i][j]=P[obs[i]][obs[j]]+R[i][j];
    inverse(C,m,inv);
    for(size_t i=0;i<5;++i)for(size_t j=0;j<m;++j)
        for(size_t k=0;k<m;++k)K[i][j]+=P[i][obs[k]]*inv[k][j];
    if(!joint)for(size_t i=0;i<5;++i) {
        bool observed=m==4?i<4:i==4;
        if(!observed)for(size_t j=0;j<m;++j)K[i][j]=0;
    }
    double expected[5];
    for(size_t i=0;i<5;++i){expected[i]=x[i];for(size_t j=0;j<m;++j)expected[i]+=K[i][j]*(z[j]-x[obs[j]]);}
    /* General covariance formula P-KHP-PH'K'+K(HPH'+R)K'. */
    double expectedP[5][5];
    for(size_t i=0;i<5;++i)for(size_t j=0;j<5;++j) {
        double value=P[i][j];
        for(size_t a=0;a<m;++a) {
            value-=K[i][a]*P[obs[a]][j]+P[i][obs[a]]*K[j][a];
            for(size_t b=0;b<m;++b)value+=K[i][a]*C[a][b]*K[j][b];
        }
        expectedP[i][j]=value;
    }
    if(m==4) {
        SrukfVnMeasurement v=vn(0,x,.1);v.x=z[0];v.y=z[1];v.heading=z[2];v.angular_velocity=z[3];
        memcpy(v.covariance,R,sizeof R);assert(srukf_process_vn(&f,&v,&work)==SRUKF_OK);
    } else {
        SrukfEncoderMeasurement e={.timestamp=0,.velocity=z[0],.variance=R[0][0]};
        assert(srukf_process_encoder(&f,&e,&work)==SRUKF_OK);
    }
    SrukfEstimate result=estimate(&f);
    for(size_t i=0;i<5;++i) {
        near(f.state[i],expected[i],1e-11);
        for(size_t j=0;j<5;++j)near(result.covariance[i][j],expectedP[i][j],1e-11);
    }
    if(!joint) {
        if(m==4){near(f.state[4],x[4],0);near(result.covariance[4][4],P[4][4],1e-12);}
        else for(size_t i=0;i<4;++i){near(f.state[i],x[i],0);near(result.covariance[i][i],P[i][i],1e-12);}
    } else {
        /* Confirms joint correction changes an unmeasured state when correlated. */
        assert(fabs(f.state[m==4?4:0]-x[m==4?4:0])>1e-5);
    }
}

static void test_packet_prediction_and_timing(bool joint) {
    const double x[5]={0,0,.2,.3,2};
    Srukf f=initial(x,.01);
    assert(f.config.correct_unmeasured_states); /* Public default. */
    f.config.correct_unmeasured_states=joint;
    Srukf predicted=f;
    assert(srukf_predict_to(&predicted,.1,&work)==SRUKF_OK);
    SrukfEncoderMeasurement e={.timestamp=.1,.velocity=5,.variance=.01};
    assert(srukf_process_encoder(&f,&e,&work)==SRUKF_OK);
    if(!joint)for(size_t i=0;i<4;++i)near(f.state[i],predicted.state[i],0);
    assert(f.state[4]>2 && f.state[4]<5);
    /* Prediction uses prior speed; joint correction then adjusts position too. */
    if(joint)assert(f.state[0]>predicted.state[0]+1e-5);
    predicted=f;assert(srukf_predict_to(&predicted,.17,&work)==SRUKF_OK);
    SrukfVnMeasurement v=vn(.17,predicted.state,.1);v.x+=.1;
    assert(srukf_process_vn(&f,&v,&work)==SRUKF_OK);
    if(!joint)near(f.state[4],predicted.state[4],0);
    else assert(f.state[4]>predicted.state[4]+1e-5);
    assert(f.state[0]>predicted.state[0]);
    /* Following E starts at VN time; no interval is lost or counted twice. */
    predicted=f;assert(srukf_predict_to(&predicted,.2,&work)==SRUKF_OK);
    e.timestamp=.2;e.velocity=f.state[4];
    assert(srukf_process_encoder(&f,&e,&work)==SRUKF_OK);
    for(size_t i=0;i<5;++i)near(f.state[i],predicted.state[i],1e-12);
    for(int joint=0;joint<=1;++joint) {
        Srukf a=initial(x,.01);a.config.correct_unmeasured_states=joint!=0;
        Srukf b=a;v=vn(.1,x,.01);v.x=.2;v.heading=.23;
        e.timestamp=.1;e.velocity=2.2;e.variance=.01;
        assert(srukf_process_encoder(&a,&e,&work)==SRUKF_OK);
        assert(srukf_process_vn(&a,&v,&work)==SRUKF_OK);
        assert(srukf_process_vn(&b,&v,&work)==SRUKF_OK);
        assert(srukf_process_encoder(&b,&e,&work)==SRUKF_OK);
        SrukfEstimate ea=estimate(&a),eb=estimate(&b);
        for(size_t i=0;i<5;++i){near(a.state[i],b.state[i],1e-10);
            for(size_t j=0;j<5;++j)near(ea.covariance[i][j],eb.covariance[i][j],1e-10);}
    }
}
static void test_gating_rollback_wrap(void) {
    double x[5]={0,0,pi-.01,0,2};
    Srukf f=initial(x,.01);
    SrukfVnMeasurement v=vn(0,x,.01);v.heading=-pi+.01;
    assert(srukf_process_vn(&f,&v,&work)==SRUKF_OK);near(fabs(f.state[2]),pi,1e-12);
    f.config.vn_gate_threshold=13.28;f.config.encoder_gate_threshold=6.63;
    Srukf before=f,pred=f;assert(srukf_predict_to(&pred,.1,&work)==SRUKF_OK);
    SrukfEncoderMeasurement e={.timestamp=.1,.velocity=1000,.variance=.01};
    assert(srukf_process_encoder(&f,&e,&work)==SRUKF_MEASUREMENT_REJECTED);
    assert(memcmp(f.state,pred.state,sizeof f.state)==0);near(f.timestamp,.1,0);
    f=before;v.timestamp=.1;v.x=1000;
    assert(srukf_process_vn(&f,&v,&work)==SRUKF_MEASUREMENT_REJECTED);
    assert(memcmp(f.state,pred.state,sizeof f.state)==0);
    before=f;e.timestamp=.2;e.variance=0;
    assert(srukf_process_encoder(&f,&e,&work)==SRUKF_INVALID_ARGUMENT);
    e.variance=-1;assert(srukf_process_encoder(&f,&e,&work)==SRUKF_INVALID_ARGUMENT);
    e.variance=NAN;assert(srukf_process_encoder(&f,&e,&work)==SRUKF_INVALID_ARGUMENT);
    v.timestamp=.2;v.covariance[0][0]=-1;
    assert(srukf_process_vn(&f,&v,&work)!=SRUKF_OK);
    v=vn(.2,x,.01);v.x=NAN;assert(srukf_process_vn(&f,&v,&work)==SRUKF_INVALID_ARGUMENT);
    e.timestamp=0;e.variance=.01;assert(srukf_process_encoder(&f,&e,&work)==SRUKF_INVALID_TIME);
    assert(srukf_predict_to(&f,DBL_MAX,&work)!=SRUKF_OK);
    assert(memcmp(&f,&before,sizeof f)==0);
    SrukfEstimate result=estimate(&f);near(result.velocity,f.state[4],0);
    srukf_reset(&f);assert(srukf_get_estimate(&f,&result)==SRUKF_NOT_INITIALIZED);
    assert(srukf_process_encoder(&f,&e,&work)==SRUKF_NOT_INITIALIZED);
    assert(srukf_process_vn(NULL,&v,&work)==SRUKF_INVALID_ARGUMENT);
}
static void test_sigma(void) {
    const double x[5]={1,2,.3,.4,5};
    Srukf f=initial(x,.03);double points[11][5];
    srukf_generate_sigma_points(f.state,f.sqrt_covariance,f.sigma_scale,points);
    double mean[5]={0},P[5][5]={{0}},sum=0;
    for(size_t k=0;k<11;++k){sum+=f.mean_weights[k];for(size_t i=0;i<5;++i)mean[i]+=f.mean_weights[k]*points[k][i];}
    near(sum,1,1e-14);
    for(size_t k=0;k<11;++k)for(size_t i=0;i<5;++i)for(size_t j=0;j<5;++j)
        P[i][j]+=f.covariance_weights[k]*(points[k][i]-mean[i])*(points[k][j]-mean[j]);
    for(size_t i=0;i<5;++i){near(mean[i],x[i],1e-13);
        for(size_t j=0;j<5;++j)near(P[i][j],i==j?.03:0,1e-13);}
    f.config.encoder_gate_threshold=NAN;assert(srukf_config_validate(&f.config)==SRUKF_INVALID_ARGUMENT);
}
int main(void) {
    assert(SRUKF_N==5 && SRUKF_SIGMA_COUNT==11 && SRUKF_MAX_MEAS==4);
    test_sigma();test_motion_and_noise();
    for(int joint=0;joint<=1;++joint){test_dense_update(SRUKF_SENSOR_VN,joint!=0);test_dense_update(SRUKF_SENSOR_ENCODER,joint!=0);}
    test_packet_prediction_and_timing(false);test_packet_prediction_and_timing(true);
    test_gating_rollback_wrap();
    puts("Core tests passed: five-state motion, sparse updates, both gain policies, Joseph covariance and timing.");
    return 0;
}
