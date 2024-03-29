#include "KFTracking.h"

CKFTracking::CKFTracking(float x0, float y0)
{
    // KF_options.method = kfEKFNaive;
    KF_options.method = kfEKFAlaDavison;

    // INIT KF STATE
    m_xkk.resize(6,0);	// State: (theta, omega, a, b, xc, yc)
    m_xkk[4] = x0;
    m_xkk[5] = y0;
    m_xkk[1] = 1;

    // Initial cov:  Large uncertainty
    m_pkk.setSize(6,6);
    m_pkk.unit();
    m_pkk *= square(5.0f);
}

CKFTracking::~CKFTracking()
{

}


void  CKFTracking::doProcess( double DeltaTime, double observationX, double observationY)
{
    m_deltaTime = (float)DeltaTime;
    m_obsX = (float)observationX;
    m_obsY = (float)observationY;

    runOneKalmanIteration();
}


void CKFTracking::OnGetAction( KFArray_ACT &u ) const
{
    u[0] = m_deltaTime;
}

void CKFTracking::OnTransitionModel(
        const KFArray_ACT &in_u,
        KFArray_VEH       &inout_x,
        bool &out_skipPrediction
        ) const
{
    // in_u[0] : Delta time
    inout_x[0] += in_u[0] * inout_x[1];
}

void CKFTracking::OnTransitionJacobian(KFMatrix_VxV  &F) const
{
    F.unit();

    F(0,1) = m_deltaTime;
}

void CKFTracking::OnTransitionNoise(KFMatrix_VxV &Q) const
{
    Q(0,0) = Q(1,1) = square(TRANSITION_MODEL_STD_ANGLE);
    Q(3,3) = Q(2,2) = square(TRANSITION_MODEL_STD_XY);
    Q(4,4) = Q(5,5) = square(TRANSITION_MODEL_STD_XY);
}

void CKFTracking::OnGetObservationNoise(KFMatrix_OxO &R) const
{
    R(0,0) = R(1,1) = square( SENSOR_NOISE_STD );
}

void CKFTracking::OnGetObservationsAndDataAssociation(
        vector_KFArray_OBS			&out_z,
        mrpt::vector_int            &out_data_association,
        const vector_KFArray_OBS	&in_all_predictions,
        const KFMatrix              &in_S,
        const vector_size_t         &in_lm_indices_in_S,
        const KFMatrix_OxO          &in_R
        )
{
    out_z.resize(1);
    out_z[0][0] = m_obsX;
    out_z[0][1] = m_obsY;

    out_data_association.clear(); // Not used
}


void CKFTracking::OnObservationModel(
        const vector_size_t       &idx_landmarks_to_predict,
        vector_KFArray_OBS	&out_predictions
        ) const
{
    out_predictions.resize(1);
    out_predictions[0][0] = m_xkk[4] + m_xkk[2] * cosf(m_xkk[0]);
    out_predictions[0][1] = m_xkk[5] + m_xkk[3] * sinf(m_xkk[0]);
}


void CKFTracking::OnObservationJacobians(
        const size_t &idx_landmark_to_predict,
        KFMatrix_OxV &Hx,
        KFMatrix_OxF &Hy
        ) const
{
    float theta = m_xkk[0];
    float a = m_xkk[2];
    float b = m_xkk[3];

    Hx.zeros();
    Hx(0, 0) = -a * sinf(theta);
    Hx(0, 2) = cosf(theta);
    Hx(0, 4) = 1;

    Hx(1, 0) = b * cosf(theta);
    Hx(1, 3) = sinf(theta);
    Hx(1, 5) = 1;
    // Hy: Not used
}


/*void CKFTracking::OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const
{
    A -= B;
    math::wrapToPiInPlace(A[0]); // The angular component
}*/

