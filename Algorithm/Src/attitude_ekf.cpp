//
// Created by 20852 on 2025/11/21.
//

#include "attitude_ekf.h"
#include <cmath>
#include <cstring>

// constants
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline float sqr(float x){ return x*x; }
static inline float norm3(const float v[3]) { return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }

AttitudeEKF::AttitudeEKF()
{
    // default noise parameters (tune for your sensors)
    gyro_noise = 0.0004f;        // (rad^2/s) -- small
    gyro_bias_noise = 1e-6f;     // (rad^2/s^3) -- bias random walk
    accel_noise = 0.02f;         // accelerometer noise (unitless after normalization)
    mag_noise = 0.05f;           // magnetometer noise (unitless after normalization)
    init();
}

void AttitudeEKF::init()
{
    // identity quaternion (no rotation)
    q[0] = 1.0f; q[1]=0.0f; q[2]=0.0f; q[3]=0.0f;
    bg[0]=bg[1]=bg[2]=0.0f;

    // P initial (small)
    memset(P,0,sizeof(P));
    for(int i=0;i<3;i++){
        P[i][i] = sqr(0.1f);   // angle uncertainty ~0.1 rad
        P[i+3][i+3] = sqr(0.01f); // bias uncertainty ~0.01 rad/s
    }

    m_ref_set = false;
    m_ref[0]=m_ref[1]=m_ref[2]=0.0f;
}

// ---------- helper implementations ----------
void AttitudeEKF::quatNormalize(float q[4]){
    float n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if(n < 1e-12f) { q[0]=1; q[1]=q[2]=q[3]=0; return; }
    q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n;
}

void AttitudeEKF::quatMultiply(const float a[4], const float b[4], float out[4]){
    // out = a * b  (Hamilton)
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void AttitudeEKF::quatIntegrate(float q[4], const float w[3], float dt){
    // integrate quaternion with angular rate w (rad/s), step dt
    // q_dot = 0.5 * Omega(w) * q
    float wx=w[0], wy=w[1], wz=w[2];
    float qd[4];
    qd[0] = -0.5f*(wx*q[1] + wy*q[2] + wz*q[3]);
    qd[1] =  0.5f*(wx*q[0] + wy*q[3] - wz*q[2]);
    qd[2] =  0.5f*( -wx*q[3] + wy*q[0] + wz*q[1]);
    qd[3] =  0.5f*( wx*q[2] - wy*q[1] + wz*q[0]);

    q[0] += qd[0]*dt;
    q[1] += qd[1]*dt;
    q[2] += qd[2]*dt;
    q[3] += qd[3]*dt;
    quatNormalize(q);
}

void AttitudeEKF::quatToDCM(const float q[4], float C[3][3]){
    // q = [w x y z]
    float w=q[0], x=q[1], y=q[2], z=q[3];
    C[0][0] = 1 - 2*(y*y + z*z);
    C[0][1] = 2*(x*y - z*w);
    C[0][2] = 2*(x*z + y*w);
    C[1][0] = 2*(x*y + z*w);
    C[1][1] = 1 - 2*(x*x + z*z);
    C[1][2] = 2*(y*z - x*w);
    C[2][0] = 2*(x*z - y*w);
    C[2][1] = 2*(y*z + x*w);
    C[2][2] = 1 - 2*(x*x + y*y);
}

void AttitudeEKF::dcmToEuler(const float C[3][3], float &roll, float &pitch, float &yaw){
    pitch = asinf( -C[2][0] );
    // guard for numerical issues
    if (fabsf(pitch - M_PI/2) < 1e-6f || fabsf(pitch + M_PI/2) < 1e-6f) {
        yaw = 0;
        roll = atan2f(-C[0][1], C[1][1]);
    } else {
        roll = atan2f(C[2][1], C[2][2]);
        yaw  = atan2f(C[1][0], C[0][0]);
    }
}

void AttitudeEKF::mat3_mul_vec(const float M[3][3], const float v[3], float out[3]){
    out[0] = M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2];
    out[1] = M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2];
    out[2] = M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2];
}

void AttitudeEKF::cross_product(const float a[3], const float b[3], float out[3]){
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

void AttitudeEKF::skew3(const float v[3], float M[3][3]){
    M[0][0]=0;    M[0][1]=-v[2]; M[0][2]= v[1];
    M[1][0]= v[2];M[1][1]=0;     M[1][2]=-v[0];
    M[2][0]=-v[1];M[2][1]= v[0]; M[2][2]=0;
}

void AttitudeEKF::mat3_add(const float A[3][3], const float B[3][3], float out[3][3]){
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) out[i][j] = A[i][j] + B[i][j];
}

void AttitudeEKF::mat3_scale(const float A[3][3], float s, float out[3][3]){
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) out[i][j] = A[i][j] * s;
}

void AttitudeEKF::mat3_copy(const float A[3][3], float out[3][3]){
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) out[i][j]=A[i][j];
}

void AttitudeEKF::mat3_inverse(const float A[3][3], float invA[3][3]){
    // compute inverse by analytical formula (adjugate / det)
    float det =
        A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1]) -
        A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0]) +
        A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (fabsf(det) < 1e-12f) {
        // singular, return identity as fallback (shouldn't happen if R sensible)
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) invA[i][j] = (i==j)?1.0f:0.0f;
        return;
    }
    float invdet = 1.0f / det;
    invA[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * invdet;
    invA[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * invdet;
    invA[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * invdet;
    invA[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * invdet;
    invA[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * invdet;
    invA[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * invdet;
    invA[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * invdet;
    invA[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * invdet;
    invA[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * invdet;
}

// ---------- EKF update ----------
void AttitudeEKF::update(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz,
                         float dt)
{
    if (dt <= 0.0f) return;

    // 1) PREDICTION
    // gyro rad/s corrected
    float omega[3] = { gx - bg[0], gy - bg[1], gz - bg[2] };

    // integrate quaternion
    quatIntegrate(q, omega, dt);

    // Build discrete F ~ I + Fc*dt, where Fc = [[-skew(omega), -I],[0,0]]
    float Fc[6][6];
    memset(Fc,0,sizeof(Fc));
    float Sk[3][3]; skew3(omega, Sk);
    // top-left = -Sk
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) Fc[i][j] = -Sk[i][j];
    // top-right = -I
    for(int i=0;i<3;i++) Fc[i][i+3] = -1.0f;
    // bottom rows zero

    // Phi = I + Fc*dt (6x6)
    float Phi[6][6];
    for(int i=0;i<6;i++){ for(int j=0;j<6;j++){ Phi[i][j] = (i==j)?1.0f:0.0f; } }
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) Phi[i][j] += Fc[i][j]*dt;

    // process noise Qd (discrete)
    float Qd[6][6]; memset(Qd,0,sizeof(Qd));
    float sg2 = gyro_noise; // variance
    float sb2 = gyro_bias_noise;
    // angle part noise ~ sg2*dt
    for(int i=0;i<3;i++) Qd[i][i] = sg2 * dt;
    for(int i=3;i<6;i++) Qd[i][i] = sb2 * dt;

    // P = Phi * P * Phi^T + Qd
    float tmp[6][6]; memset(tmp,0,sizeof(tmp));
    // tmp = Phi * P
    for(int i=0;i<6;i++) for(int j=0;j<6;j++){
        float s=0;
        for(int k=0;k<6;k++) s += Phi[i][k]*P[k][j];
        tmp[i][j]=s;
    }
    float Ppred[6][6]; memset(Ppred,0,sizeof(Ppred));
    // Ppred = tmp * Phi^T
    for(int i=0;i<6;i++) for(int j=0;j<6;j++){
        float s=0;
        for(int k=0;k<6;k++) s += tmp[i][k]*Phi[j][k];
        Ppred[i][j] = s + Qd[i][j];
    }

    // 2) MEASUREMENT UPDATE USING ACCEL (if valid)
    // normalize accel
    float acc_vec[3] = {ax, ay, az};
    float a_norm = norm3(acc_vec);
    if(a_norm > 1e-6f){
        float a_meas[3] = {ax/a_norm, ay/a_norm, az/a_norm};

        // expected accel in body frame = C^T * g   (g = [0,0,1] in nav)
        float C[3][3]; quatToDCM(q,C);
        // expected = third column of C^T i.e. C[2][0..2] (gravity direction in body)
        float exp_a[3] = { C[2][0], C[2][1], C[2][2] }; // gravity direction in body frame

        // innovation y = a_meas - exp_a
        float y[3] = { a_meas[0] - exp_a[0], a_meas[1] - exp_a[1], a_meas[2] - exp_a[2] };

        // H_acc = [ -skew(exp_a) , 0 ]  (3x6)
        float H[3][6]; memset(H,0,sizeof(H));
        float Sexp[3][3]; skew3(exp_a, Sexp);
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) H[i][j] = -Sexp[i][j];

        // S = H * Ppred * H^T + R
        float HP[3][6]; memset(HP,0,sizeof(HP));
        for(int i=0;i<3;i++) for(int j=0;j<6;j++){
            float s=0;
            for(int k=0;k<6;k++) s += H[i][k]*Ppred[k][j];
            HP[i][j]=s;
        }
        float Smat[3][3]; memset(Smat,0,sizeof(Smat));
        for(int i=0;i<3;i++) for(int j=0;j<3;j++){
            float s=0;
            for(int k=0;k<6;k++) s += HP[i][k]*H[j][k];
            Smat[i][j] = s;
        }
        // add measurement noise R = accel_noise * I
        for(int i=0;i<3;i++) Smat[i][i] += accel_noise;

        // invert Smat (3x3)
        float SmatInv[3][3]; mat3_inverse(Smat, SmatInv);

        // K = Ppred * H^T * SmatInv  -> (6x3)
        float PT[6][3]; memset(PT,0,sizeof(PT));
        for(int i=0;i<6;i++) for(int j=0;j<3;j++){
            float s=0;
            for(int k=0;k<6;k++) s += Ppred[i][k]*H[j][k];
            PT[i][j] = s;
        }
        float K[6][3]; memset(K,0,sizeof(K));
        for(int i=0;i<6;i++) for(int j=0;j<3;j++){
            float s=0;
            for(int k=0;k<3;k++) s += PT[i][k]*SmatInv[k][j];
            K[i][j]=s;
        }

        // delta_x = K * y  (6x1)
        float dx[6]; memset(dx,0,sizeof(dx));
        for(int i=0;i<6;i++){
            float s=0;
            for(int k=0;k<3;k++) s += K[i][k]*y[k];
            dx[i]=s;
        }

        // apply correction: small angle approx -> update quaternion: q <- delta_q * q
        float dtheta[3] = { dx[0], dx[1], dx[2] };
        float dq[4];
        dq[0] = 1.0f;
        dq[1] = 0.5f * dtheta[0];
        dq[2] = 0.5f * dtheta[1];
        dq[3] = 0.5f * dtheta[2];
        float q_new[4];
        quatMultiply(dq, q, q_new);
        memcpy(q, q_new, sizeof(q));
        quatNormalize(q);

        // update bias
        bg[0] += dx[3];
        bg[1] += dx[4];
        bg[2] += dx[5];

        // update P = (I - K H) Ppred
        float I_KH[6][6]; memset(I_KH,0,sizeof(I_KH));
        // compute KH (6x6) = K(6x3) * H(3x6)
        float KH[6][6]; memset(KH,0,sizeof(KH));
        for(int i=0;i<6;i++) for(int j=0;j<6;j++){
            float s=0;
            for(int k=0;k<3;k++) s += K[i][k]*H[k][j];
            KH[i][j]=s;
        }
        for(int i=0;i<6;i++) for(int j=0;j<6;j++) I_KH[i][j] = ((i==j)?1.0f:0.0f) - KH[i][j];

        float Pnew[6][6]; memset(Pnew,0,sizeof(Pnew));
        for(int i=0;i<6;i++) for(int j=0;j<6;j++){
            float s=0;
            for(int k=0;k<6;k++) s += I_KH[i][k]*Ppred[k][j];
            Pnew[i][j] = s;
        }
        memcpy(P, Pnew, sizeof(P));
    } else {
        // no accel update: set P = Ppred
        memcpy(P, Ppred, sizeof(P));
    }

    // 3) MAG update (if available)
    // === FIXED: use magnetometer inputs (mx,my,mz), not accel ===
    float mag_vec[3] = {mx, my, mz};
    float m_norm = norm3(mag_vec);
    if(m_norm > 1e-6f){
        float m_meas[3] = { mx/m_norm, my/m_norm, mz/m_norm };

        // set reference magnetic vector if not set
        if(!m_ref_set){
            // compute nav-frame m_ref by rotating measured by C (body->nav): m_ref = C * m_meas
            float C[3][3]; quatToDCM(q,C);
            float mnav[3];
            mat3_mul_vec(C, m_meas, mnav);
            float nrm = norm3(mnav);
            if(nrm > 1e-6f){
                m_ref[0] = mnav[0]/nrm;
                m_ref[1] = mnav[1]/nrm;
                m_ref[2] = mnav[2]/nrm;
                m_ref_set = true;
            }
        }

        if(m_ref_set){
            // expected mag in body = C^T * m_ref
            float C[3][3]; quatToDCM(q,C);
            float exp_m[3]; // C^T * m_ref -> exp_m[i] = sum_j C[j][i]*m_ref[j]
            float tmpv[3] = { m_ref[0], m_ref[1], m_ref[2] };
            exp_m[0] = C[0][0]*tmpv[0] + C[1][0]*tmpv[1] + C[2][0]*tmpv[2];
            exp_m[1] = C[0][1]*tmpv[0] + C[1][1]*tmpv[1] + C[2][1]*tmpv[2];
            exp_m[2] = C[0][2]*tmpv[0] + C[1][2]*tmpv[1] + C[2][2]*tmpv[2];

            float y[3] = { m_meas[0] - exp_m[0], m_meas[1] - exp_m[1], m_meas[2] - exp_m[2] };

            float H[3][6]; memset(H,0,sizeof(H));
            float Sexp[3][3]; skew3(exp_m, Sexp);
            for(int i=0;i<3;i++) for(int j=0;j<3;j++) H[i][j] = -Sexp[i][j];

            // S = H * P * H^T + R
            float HP[3][6]; memset(HP,0,sizeof(HP));
            for(int i=0;i<3;i++) for(int j=0;j<6;j++){
                float s=0;
                for(int k=0;k<6;k++) s += H[i][k]*P[k][j];
                HP[i][j]=s;
            }
            float Smat[3][3]; memset(Smat,0,sizeof(Smat));
            for(int i=0;i<3;i++) for(int j=0;j<3;j++){
                float s=0;
                for(int k=0;k<6;k++) s += HP[i][k]*H[j][k];
                Smat[i][j] = s;
            }
            for(int i=0;i<3;i++) Smat[i][i] += mag_noise;

            float SmatInv[3][3]; mat3_inverse(Smat, SmatInv);

            // K = P * H^T * SmatInv
            float PT[6][3]; memset(PT,0,sizeof(PT));
            for(int i=0;i<6;i++) for(int j=0;j<3;j++){
                float s=0;
                for(int k=0;k<6;k++) s += P[i][k]*H[j][k];
                PT[i][j] = s;
            }
            float K[6][3]; memset(K,0,sizeof(K));
            for(int i=0;i<6;i++) for(int j=0;j<3;j++){
                float s=0;
                for(int k=0;k<3;k++) s += PT[i][k]*SmatInv[k][j];
                K[i][j]=s;
            }

            // delta_x = K*y
            float dx[6]; memset(dx,0,sizeof(dx));
            for(int i=0;i<6;i++){
                float s=0;
                for(int k=0;k<3;k++) s += K[i][k]*y[k];
                dx[i]=s;
            }

            float dtheta[3] = { dx[0], dx[1], dx[2] };
            float dq[4]; dq[0]=1.0f; dq[1]=0.5f*dtheta[0]; dq[2]=0.5f*dtheta[1]; dq[3]=0.5f*dtheta[2];
            float q_new[4]; quatMultiply(dq,q,q_new); memcpy(q,q_new,sizeof(q)); quatNormalize(q);

            bg[0]+=dx[3]; bg[1]+=dx[4]; bg[2]+=dx[5];

            // P <- (I-KH)P
            float KH[6][6]; memset(KH,0,sizeof(KH));
            for(int i=0;i<6;i++) for(int j=0;j<6;j++){
                float s=0;
                for(int k=0;k<3;k++) s += K[i][k]*H[k][j];
                KH[i][j]=s;
            }
            float I_KH[6][6]; memset(I_KH,0,sizeof(I_KH));
            for(int i=0;i<6;i++) for(int j=0;j<6;j++) I_KH[i][j] = ((i==j)?1.0f:0.0f) - KH[i][j];
            float Pnew[6][6]; memset(Pnew,0,sizeof(Pnew));
            for(int i=0;i<6;i++) for(int j=0;j<6;j++){
                float s=0;
                for(int k=0;k<6;k++) s += I_KH[i][k]*P[k][j];
                Pnew[i][j]=s;
            }
            memcpy(P,Pnew,sizeof(P));
        }
    }

    // done
}

EKF_Output AttitudeEKF::getOutput() const {
    EKF_Output out;
    out.q0 = q[0]; out.q1 = q[1]; out.q2 = q[2]; out.q3 = q[3];
    float C[3][3]; const_cast<AttitudeEKF*>(this)->quatToDCM(q,C);
    float roll,pitch,yaw; AttitudeEKF::dcmToEuler(C, roll, pitch, yaw);
    out.roll = roll; out.pitch = pitch; out.yaw = yaw;
    return out;
}
