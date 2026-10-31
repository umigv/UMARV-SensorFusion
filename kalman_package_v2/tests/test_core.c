#include "kalman_interface.h"
#include "kalman_internal.h"
#include "kalman_math.h"
#include "kalman_workspace.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void close_to(double a, double b) {
    if (!(fabs(a-b) < 1e-9 * (1+fabs(b)))) {
        fprintf(stderr, "mismatch %.17g != %.17g\n", a, b);
        assert(0);
    }
}
static Srukf make_filter(void) {
    Srukf f; SrukfConfig c; srukf_config_default(&c);
    double x[5]={10,20,1,0.1,0.01};
    double p[5][5]={{4,0.5,0.2,0,0},{0.5,3,0,0,0},
                   {0.2,0,1,0,0},{0,0,0,0.01,0},{0,0,0,0,0.01}};
    assert(srukf_init(&f,&c,0,x,p)==SRUKF_OK); return f;
}
static void test_math(void) {
    SrukfWorkspace w={0};
    for (size_t n=1;n<=5;++n) {
        double a[80]={0}, gram[25]={0}, l[25]={0}, p[25]={0};
        for(size_t i=0;i<n;++i) for(size_t j=0;j<n+2;++j)
            a[i*16+j]=sin((double)(i*7+j*3+1))+(i==j?2:0);
        for(size_t i=0;i<n;++i) for(size_t j=0;j<n;++j)
            for(size_t k=0;k<n+2;++k) gram[i*5+j]+=a[i*16+k]*a[j*16+k];
        assert(srmat_qr_lower_factor(a,n,n+2,16,l,5,&w)==SRUKF_OK);
        srmat_covariance_from_factor(l,n,5,p,5);
        for(size_t i=0;i<n;++i) for(size_t j=0;j<n;++j) close_to(p[i*5+j],gram[i*5+j]);
        double u[5]={0}, orig[5]={0}, saved[25]; memcpy(saved,l,sizeof l);
        for(size_t i=0;i<n;++i) u[i]=orig[i]=0.1*(i+1);
        assert(srmat_cholesky_rank1(l,n,5,u,1)==SRUKF_OK);
        srmat_covariance_from_factor(l,n,5,p,5);
        for(size_t i=0;i<n;++i) for(size_t j=0;j<n;++j) close_to(p[i*5+j],gram[i*5+j]+orig[i]*orig[j]);
        memcpy(u,orig,sizeof u); assert(srmat_cholesky_rank1(l,n,5,u,-1)==SRUKF_OK);
        for(size_t i=0;i<n;++i) for(size_t j=0;j<=i;++j) close_to(l[i*5+j],saved[i*5+j]);
        double b[5]={0},x[5]={0};
        for(size_t i=0;i<n;++i) for(size_t j=0;j<=i;++j) b[i]+=l[i*5+j]*(j+1);
        assert(srmat_solve_lower(l,n,5,b,x)==SRUKF_OK);
        for(size_t i=0;i<n;++i) close_to(x[i],i+1);
        memset(b,0,sizeof b);
        for(size_t i=0;i<n;++i) for(size_t j=i;j<n;++j) b[i]+=l[j*5+i]*(j+1);
        assert(srmat_solve_lower_transpose(l,n,5,b,b)==SRUKF_OK);
        for(size_t i=0;i<n;++i) close_to(b[i],i+1);
    }
    double l[4]={1,0,0,1}, save[4]; memcpy(save,l,sizeof l);
    double u[2]={2,0}; assert(srmat_cholesky_rank1(l,2,2,u,-1)==SRUKF_NUMERICAL_FAILURE);
    assert(memcmp(l,save,sizeof l)==0 && u[0]==2);
    double dependent[6]={1,2,3,2,4,6};
    assert(srmat_qr_lower_factor(dependent,2,3,3,l,2,&w)==SRUKF_NUMERICAL_FAILURE);
    double bad[4]={1,2,3,1}; assert(srmat_cholesky_lower(bad,2,2,l,2)!=SRUKF_OK);
    assert(!srmat_is_finite(NULL,1,1,1));
}
static void test_update(void) {
    Srukf f=make_filter(); SrukfWorkspace w={0}; SrukfEstimate before,after;
    assert(srukf_get_estimate(&f,&before)==SRUKF_OK);
    double z[3]={11,19,0},r[3][3]={{2,0.2,0},{0.2,1,0},{0,0,0}};
    /* Independent dense Kalman reference for the linear GPS observation. */
    double a=6,b=0.7,d=4,det=a*d-b*b,k[5][2];
    for(size_t i=0;i<5;++i) {
        double p0=before.covariance[i][0],p1=before.covariance[i][1];
        k[i][0]=(p0*d-p1*b)/det; k[i][1]=(p1*a-p0*b)/det;
    }
    assert(srukf_measurement_update(&f,SRUKF_SENSOR_GPS,z,r,0,&w)==SRUKF_OK);
    assert(srukf_get_estimate(&f,&after)==SRUKF_OK);
    double initial[5]={10,20,1,0.1,0.01};
    for(size_t i=0;i<5;++i) {
        close_to(f.state[i],initial[i]+k[i][0]-k[i][1]);
        for(size_t j=0;j<5;++j) close_to(after.covariance[i][j],before.covariance[i][j]-k[i][0]*before.covariance[0][j]-k[i][1]*before.covariance[1][j]);
    }
    Srukf save=f;
    SrukfGpsMeasurement g={.timestamp=0.1,.x=1e6,.y=1e6,.covariance={{1,0},{0,1}}};
    assert(srukf_process_gps(&f,&g,&w)==SRUKF_MEASUREMENT_REJECTED); close_to(f.timestamp,0.1);
    save=f; g.timestamp=0; assert(srukf_process_gps(&f,&g,&w)==SRUKF_INVALID_TIME); assert(memcmp(&f,&save,sizeof f)==0);
    g.timestamp=1; g.covariance[0][0]=-1; assert(srukf_process_gps(&f,&g,&w)!=SRUKF_OK); assert(memcmp(&f,&save,sizeof f)==0);
    assert(srukf_predict_to(&f,INFINITY,&w)==SRUKF_INVALID_TIME);
    assert(srukf_predict_to(&f,1e100,&w)==SRUKF_INVALID_TIME);
    assert(memcmp(&f,&save,sizeof f)==0);
    srukf_reset(&f); assert(!f.initialized); assert(srukf_get_estimate(&f,&after)==SRUKF_NOT_INITIALIZED);
}
static void test_motion(void) {
    const double pi=acos(-1.0);
    double x[5]={0,0,1,0,pi/2},y[5]; srukf_motion_model(x,1,y);
    close_to(y[0],2/pi);close_to(y[1],2/pi);
    x[4]=0;srukf_motion_model(x,1,y);close_to(y[0],1);close_to(y[1],0);
    double pts[2]={pi-0.01,-pi+0.01},weights[2]={0.5,0.5},mean;
    assert(srukf_weighted_mean(pts,2,1,1,weights,0,&mean)==SRUKF_OK);close_to(fabs(mean),pi);
    Srukf f=make_filter(); SrukfWorkspace w={0};
    f.state[3]=pi-0.01;
    SrukfOdometryMeasurement o={.timestamp=0,.velocity=1,.heading=-pi+0.01,.angular_velocity=0.01,.covariance={{0.01,0,0},{0,0.01,0},{0,0,0.01}}};
    assert(srukf_process_odometry(&f,&o,&w)==SRUKF_OK);
    double q[5][5];assert(srukf_process_noise_factor(&f.config,x,0.2,q)==SRUKF_OK);
    double q00=0,q02=0,q22=0;for(size_t k=0;k<5;++k){q00+=q[0][k]*q[0][k];q02+=q[0][k]*q[2][k];q22+=q[2][k]*q[2][k];}
    close_to(q00,0.25*0.008/3);close_to(q02,0.25*0.04/2);close_to(q22,0.25*0.2);
}
int main(void){test_math();test_update();test_motion();puts("Core numerical/API tests passed");return 0;}
