#include <mrpt/bayes/CKalmanFilterCapable.h>

using namespace mrpt::bayes;

#define SENSOR_NOISE_STD 		0.3f
#define DELTA_TIME                  	0.1f

#define VEHICLE_INITIAL_X			4.0f
#define VEHICLE_INITIAL_Y			4.0f
#define VEHICLE_INITIAL_V           1.0f
#define VEHICLE_INITIAL_W           DEG2RAD(20.0f)

#define TRANSITION_MODEL_STD_XY 	0.03f
#define TRANSITION_MODEL_STD_VXY 	0.20f

extern template class CKalmanFilterCapable<4, 2, 0, 1>;

class CKFTracking : public CKalmanFilterCapable<4, 2, 0, 1>
{
public:
    CKFTracking( );
    virtual ~CKFTracking();

    void  doProcess(double DeltaTime, double observationX, double observationY );

    void getState( KFVector &xkk, KFMatrix &pkk)
    {
        xkk = m_xkk;
        pkk = m_pkk;
    }

protected:

    float m_obsX, m_obsY;
    float m_deltaTime;

    void OnGetAction( KFArray_ACT &out_u ) const;


    void OnTransitionModel(
            const KFArray_ACT &in_u,
            KFArray_VEH       &inout_x,
            bool &out_skipPrediction
            ) const;

    void OnTransitionJacobian(KFMatrix_VxV  &out_F ) const;

    void OnTransitionNoise(KFMatrix_VxV &out_Q ) const;

    void OnGetObservationNoise(KFMatrix_OxO &out_R) const;

    void OnGetObservationsAndDataAssociation(
            vector_KFArray_OBS			&out_z,
            mrpt::vector_int            &out_data_association,
            const vector_KFArray_OBS	&in_all_predictions,
            const KFMatrix              &in_S,
            const vector_size_t         &in_lm_indices_in_S,
            const KFMatrix_OxO          &in_R
            );

    void OnObservationModel(
            const vector_size_t &idx_landmarks_to_predict,
            vector_KFArray_OBS  &out_predictions
            ) const;

    void OnObservationJacobians(
            const size_t &idx_landmark_to_predict,
            KFMatrix_OxV &Hx,
            KFMatrix_OxF &Hy
            ) const;

    // void OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const;
};

