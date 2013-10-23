#include "KFTracking.h"

CKFTracking::CKFTracking(float x0, float y0)
{
    // KF_options.method = kfEKFNaive;
    KF_options.method = kfEKFAlaDavison;

    // INIT KF STATE
    m_xkk.resize(4,0);	// State: (x,y,heading,v,w)
    m_xkk[0]= x0;
    m_xkk[1]= y0;
    m_xkk[2]=-VEHICLE_INITIAL_V;
    m_xkk[3]=0;

    // Initial cov:  Large uncertainty
    m_pkk.setSize(4,4);
    m_pkk.unit();
    m_pkk(0,0)=
            m_pkk(1,1)= square( 5.0f );
    m_pkk(2,2)=
            m_pkk(3,3)= square( 1.0f );
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
    // in_out_x: [0]:x  [1]:y  [2]:vx  [3]: vy
    inout_x[0] += in_u[0] * inout_x[2];
    inout_x[1] += in_u[0] * inout_x[3];

}

void CKFTracking::OnTransitionJacobian(KFMatrix_VxV  &F) const
{
    F.unit();

    F(0,2) = m_deltaTime;
    F(1,3) = m_deltaTime;
}

void CKFTracking::OnTransitionNoise(KFMatrix_VxV &Q) const
{
    Q(0,0) = Q(1,1) = square( TRANSITION_MODEL_STD_XY );
    Q(2,2) = Q(3,3) = square( TRANSITION_MODEL_STD_VXY );
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
    out_predictions[0][0] = m_xkk[0];
    out_predictions[0][1] = m_xkk[1];
}


void CKFTracking::OnObservationJacobians(
        const size_t &idx_landmark_to_predict,
        KFMatrix_OxV &Hx,
        KFMatrix_OxF &Hy
        ) const
{
    Hx.zeros();
    Hx(0,0) = 1;
    Hx(1,1) = 1;

    // Hy: Not used
}

/*
void CKFTracking::OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const
{
    A -= B;
    math::wrapToPiInPlace(A[0]); // The angular component
}*/

