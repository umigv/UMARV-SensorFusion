#include "kalman_interface.h"
#include "kalman_internal.h"
#include "kalman_math.h"
#include "kalman_workspace.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const double pi = 3.14159265358979323846;
static SrukfWorkspace workspace;

static void near(double a, double b, double tolerance)
{
    if (!(fabs(a-b) <= tolerance)) {
        fprintf(stderr, "mismatch %.17g != %.17g (tol %.3g)\n", a,b,tolerance);
        assert(0);
    }
}

static Srukf initialized(const double x[4], double variance)
{
    Srukf f = {0}; SrukfConfig c;
    srukf_config_default(&c);
    c.vn_gate_threshold = 0;
    c.linear_accel_noise_density = 0;
    c.angular_accel_noise_density = 0;
    double P[4][4] = {{0}};
    for (size_t i=0; i<4; ++i) P[i][i]=variance;
    assert(srukf_init(&f,&c,0,x,P)==SRUKF_OK);
    return f;
}

static SrukfVnMeasurement vn_at(double time, const double x[4], double variance)
{
    SrukfVnMeasurement m = {0};
    m.timestamp=time; m.x=x[0]; m.y=x[1];
    m.heading=x[2]; m.angular_velocity=x[3];
    for (size_t i=0;i<4;++i) m.covariance[i][i]=variance;
    return m;
}

static void check_spd(const Srukf *f)
{
    SrukfEstimate e;
    assert(srukf_get_estimate(f,&e)==SRUKF_OK);
    double P[16], L[16];
    for (size_t i=0;i<4;++i) for (size_t j=0;j<4;++j) {
        assert(isfinite(e.covariance[i][j]));
        near(e.covariance[i][j],e.covariance[j][i],1e-12);
        P[i*4+j]=e.covariance[i][j];
    }
    assert(srukf_wrap_angle(e.heading)==e.heading);
    assert(srmat_cholesky_lower(P,4,4,L,4)==SRUKF_OK);
}

static void test_motion_and_encoder(void)
{
    const double initial[4]={2,-1,0.4,0.6};
    double out[4];
    srukf_motion_model(initial,0.3,3.0,out);
    near(out[0],2+0.9*cos(0.49),1e-14);
    near(out[1],-1+0.9*sin(0.49),1e-14);
    near(out[2],0.58,1e-14); near(out[3],0.6,0);
    srukf_motion_model(initial,0.3,-3.0,out);
    near(out[0],2-0.9*cos(0.49),1e-14);
    srukf_motion_model(initial,0.3,0,out);
    near(out[0],2,0); near(out[1],-1,0); near(out[2],0.58,1e-14);
    double wrapped[4]={0,0,pi-0.01,1};
    srukf_motion_model(wrapped,0.02,2,wrapped); /* in-place */
    near(wrapped[2],-pi+0.01,1e-14);

    Srukf f=initialized(initial,1e-10);
    SrukfEncoderMeasurement e={.timestamp=0.3,.velocity=3,.variance=0.04};
    assert(srukf_process_encoder(&f,&e,&workspace)==SRUKF_OK);
    near(f.state[0],2+0.9*cos(0.49),1e-9);
    near(f.state[1],-1+0.9*sin(0.49),1e-9);
    near(f.state[2],0.58,1e-12); near(f.state[3],0.6,1e-12);
    near(f.timestamp,0.3,0); check_spd(&f);
    Srukf saved=f;
    e.velocity=-100;
    assert(srukf_process_encoder(&f,&e,&workspace)==SRUKF_OK);
    assert(memcmp(&saved,&f,sizeof f)==0); /* equal time has no effect */
    e.timestamp=0.6; e.velocity=0; e.variance=0;
    assert(srukf_process_encoder(&f,&e,&workspace)==SRUKF_OK);
    near(f.state[0],saved.state[0],1e-10); /* never reuses previous speed */
    near(f.state[1],saved.state[1],1e-10);
    near(f.state[2],0.76,1e-10);
}

static void test_noise(void)
{
    const double x[4]={0,0,0.2,0.8};
    SrukfConfig c; srukf_config_default(&c);
    const double dt=0.3, rv=0.09;
    double L[4][4], P[4][4]={{0}};
    assert(srukf_process_noise_factor(&c,x,dt,rv,L)==SRUKF_OK);
    for (size_t i=0;i<4;++i) for (size_t j=0;j<4;++j)
        for (size_t k=0;k<4;++k) P[i][j]+=L[i][k]*L[j][k];
    const double d=rv*dt*dt+c.linear_accel_noise_density*dt*dt*dt/3;
    const double angle=x[2]+0.5*x[3]*dt;
    near(P[0][0],d*cos(angle)*cos(angle),1e-14);
    near(P[0][1],d*cos(angle)*sin(angle),1e-14);
    near(P[1][1],d*sin(angle)*sin(angle),1e-14);
    near(P[2][2],c.angular_accel_noise_density*dt*dt*dt/3,1e-14);
    near(P[2][3],c.angular_accel_noise_density*dt*dt/2,1e-14);
    near(P[3][3],c.angular_accel_noise_density*dt,1e-14);
    assert(srukf_process_noise_factor(&c,x,0,rv,L)==SRUKF_OK);
    for (size_t i=0;i<4;++i) for (size_t j=0;j<4;++j) near(L[i][j],0,0);
    Srukf a=initialized(x,0.001), b=a;
    SrukfEncoderMeasurement e={.timestamp=0.1,.velocity=3,.variance=0};
    assert(srukf_process_encoder(&a,&e,&workspace)==SRUKF_OK);
    e.variance=4;
    assert(srukf_process_encoder(&b,&e,&workspace)==SRUKF_OK);
    SrukfEstimate ea,eb;
    assert(srukf_get_estimate(&a,&ea)==SRUKF_OK);
    assert(srukf_get_estimate(&b,&eb)==SRUKF_OK);
    near(eb.covariance[0][0]-ea.covariance[0][0],
         0.04*cos(0.24)*cos(0.24),1e-12);
    near(eb.covariance[0][1]-ea.covariance[0][1],
         0.04*cos(0.24)*sin(0.24),1e-12);
}

/* Independent dense inverse for a test reference, never used by the library. */
static void inverse4(const double A[4][4], double inverse[4][4])
{
    double B[4][8]={{0}};
    for(size_t i=0;i<4;++i) {
        for(size_t j=0;j<4;++j) B[i][j]=A[i][j];
        B[i][i+4]=1;
    }
    for(size_t k=0;k<4;++k) {
        size_t pivot=k;
        for(size_t i=k+1;i<4;++i) if(fabs(B[i][k])>fabs(B[pivot][k])) pivot=i;
        for(size_t j=0;j<8;++j) {double t=B[k][j];B[k][j]=B[pivot][j];B[pivot][j]=t;}
        double d=B[k][k];assert(fabs(d)>1e-14);
        for(size_t j=0;j<8;++j) B[k][j]/=d;
        for(size_t i=0;i<4;++i) if(i!=k) {
            double m=B[i][k];for(size_t j=0;j<8;++j) B[i][j]-=m*B[k][j];
        }
    }
    for(size_t i=0;i<4;++i) for(size_t j=0;j<4;++j) inverse[i][j]=B[i][j+4];
}

static void test_vn_against_dense_reference(void)
{
    const double x[4]={1,-2,0.4,0.1};
    const double z[4]={1.2,-1.7,0.5,0.08};
    const double P[4][4]={{.5,.07,.01,.002},{.07,.4,-.02,.003},
                         {.01,-.02,.03,.001},{.002,.003,.001,.02}};
    Srukf f; SrukfConfig c;srukf_config_default(&c);c.vn_gate_threshold=0;
    assert(srukf_init(&f,&c,0,x,P)==SRUKF_OK);
    SrukfVnMeasurement m=vn_at(3,z,0.1);
    m.covariance[0][1]=m.covariance[1][0]=0.02;
    m.covariance[2][3]=m.covariance[3][2]=0.004;
    double innovation[4][4], inv[4][4], K[4][4]={{0}};
    for(size_t i=0;i<4;++i) for(size_t j=0;j<4;++j)
        innovation[i][j]=P[i][j]+m.covariance[i][j];
    inverse4(innovation,inv);
    for(size_t i=0;i<4;++i) for(size_t j=0;j<4;++j)
        for(size_t k=0;k<4;++k) K[i][j]+=P[i][k]*inv[k][j];
    assert(srukf_process_vn(&f,&m,&workspace)==SRUKF_OK);
    SrukfEstimate result;assert(srukf_get_estimate(&f,&result)==SRUKF_OK);
    for(size_t i=0;i<4;++i) {
        double expected=x[i];
        for(size_t j=0;j<4;++j) expected+=K[i][j]*(z[j]-x[j]);
        near(f.state[i],expected,1e-11);
        for(size_t j=0;j<4;++j) {
            double expected_p=P[i][j];
            for(size_t k=0;k<4;++k) expected_p-=K[i][k]*P[k][j];
            near(result.covariance[i][j],expected_p,1e-11);
        }
    }
    near(f.timestamp,3,0);check_spd(&f);
    /* At a later VN time, omega must not advance heading (identity prediction). */
    const double same[4]={f.state[0],f.state[1],f.state[2],f.state[3]};
    m=vn_at(30,same,0.1);
    assert(srukf_process_vn(&f,&m,&workspace)==SRUKF_OK);
    for(size_t i=0;i<4;++i) near(f.state[i],same[i],1e-11);
    /* E interval starts at this committed VN timestamp, not the previous E. */
    SrukfEncoderMeasurement e={.timestamp=30.01,.velocity=0,.variance=0};
    assert(srukf_process_encoder(&f,&e,&workspace)==SRUKF_OK);
    near(f.state[2],same[2]+same[3]*0.01,1e-11);
}

static void test_wrap_rejection_and_rollback(void)
{
    const double x[4]={0,0,pi-0.01,0};
    Srukf f=initialized(x,0.01);
    const double z[4]={0,0,-pi+0.01,0};
    SrukfVnMeasurement m=vn_at(0,z,0.01);
    assert(srukf_process_vn(&f,&m,&workspace)==SRUKF_OK);
    near(fabs(f.state[2]),pi,1e-12);
    f.config.vn_gate_threshold=13.28;
    Srukf before=f;
    m.x=1e5;m.timestamp=1;
    assert(srukf_process_vn(&f,&m,&workspace)==SRUKF_MEASUREMENT_REJECTED);
    near(f.timestamp,1,0);
    assert(memcmp(f.state,before.state,sizeof f.state)==0);
    assert(memcmp(f.sqrt_covariance,before.sqrt_covariance,sizeof f.sqrt_covariance)==0);
    before=f;
    m.timestamp=2;m.covariance[0][0]=-1;
    assert(srukf_process_vn(&f,&m,&workspace)!=SRUKF_OK);
    assert(memcmp(&f,&before,sizeof f)==0);
    m=vn_at(0,z,0.01);
    assert(srukf_process_vn(&f,&m,&workspace)==SRUKF_INVALID_TIME);
    assert(memcmp(&f,&before,sizeof f)==0);
    SrukfEncoderMeasurement e={.timestamp=2,.velocity=2,.variance=-1};
    assert(srukf_process_encoder(&f,&e,&workspace)==SRUKF_INVALID_ARGUMENT);
    e.variance=NAN;
    assert(srukf_process_encoder(&f,&e,&workspace)==SRUKF_INVALID_ARGUMENT);
    e.variance=0;e.timestamp=0;
    assert(srukf_process_encoder(&f,&e,&workspace)==SRUKF_INVALID_TIME);
    e.timestamp=DBL_MAX;e.velocity=DBL_MAX;
    assert(srukf_process_encoder(&f,&e,&workspace)!=SRUKF_OK);
    assert(memcmp(&f,&before,sizeof f)==0);
    m=vn_at(2,z,0.01);m.heading=NAN;
    assert(srukf_process_vn(&f,&m,&workspace)==SRUKF_INVALID_ARGUMENT);
    assert(memcmp(&f,&before,sizeof f)==0);
    srukf_reset(&f);
    assert(srukf_process_vn(&f,&m,&workspace)==SRUKF_NOT_INITIALIZED);
    SrukfEstimate result;
    assert(srukf_get_estimate(&f,&result)==SRUKF_NOT_INITIALIZED);
    assert(srukf_process_encoder(&f,NULL,&workspace)==SRUKF_INVALID_ARGUMENT);
    assert(srukf_process_vn(NULL,&m,&workspace)==SRUKF_INVALID_ARGUMENT);
}

static void test_weights_and_covariance(void)
{
    const double x[4]={1,2,0.4,0.1};
    Srukf f=initialized(x,0.03);
    double points[SRUKF_SIGMA_COUNT][4];
    srukf_generate_sigma_points(f.state,f.sqrt_covariance,f.sigma_scale,points);
    double mean[4]={0}, P[4][4]={{0}}, sum=0;
    for(size_t k=0;k<SRUKF_SIGMA_COUNT;++k) {
        sum+=f.mean_weights[k];
        for(size_t i=0;i<4;++i) mean[i]+=f.mean_weights[k]*points[k][i];
    }
    near(sum,1,1e-14);
    for(size_t k=0;k<SRUKF_SIGMA_COUNT;++k)
        for(size_t i=0;i<4;++i) for(size_t j=0;j<4;++j)
            P[i][j]+=f.covariance_weights[k]*(points[k][i]-mean[i])*(points[k][j]-mean[j]);
    for(size_t i=0;i<4;++i) {
        near(mean[i],x[i],1e-14);
        for(size_t j=0;j<4;++j) near(P[i][j],i==j?0.03:0,1e-14);
    }
    f.config.alpha=0;assert(srukf_config_validate(&f.config)==SRUKF_INVALID_ARGUMENT);
    assert(srukf_measurement_dimension(SRUKF_SENSOR_VN)==4);
    assert(srukf_measurement_dimension((SrukfSensor)99)==0);
}

int main(void)
{
    assert(SRUKF_N==4 && SRUKF_SIGMA_COUNT==9 && SRUKF_MAX_MEAS==4);
    test_weights_and_covariance();
    test_motion_and_encoder(); test_noise(); test_vn_against_dense_reference();
    test_wrap_rejection_and_rollback();
    puts("Core tests passed: four-state prediction, covariance, VN correction, timing and rollback.");
    return 0;
}
