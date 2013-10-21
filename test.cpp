#include <mrpt/base.h>
#include <mrpt/gui.h>

#include "KFTracking.h"

using namespace mrpt::gui;
using namespace mrpt::random;
using namespace std;

//#define SAVE_GT_LOGS

void TestBayesianTracking()
{
    randomGenerator.randomize();
    
    CDisplayWindowPlots		winEKF("Tracking - Extended Kalman Filter",450,400);
    
    winEKF.setPos(10,10);
    winEKF.axis(-2,20,-10,10); winEKF.axis_equal();
    
    // Create EKF
    // ----------------------
    CKFTracking 	EKF;
    EKF.KF_options.method = kfEKFNaive;
    
    EKF.KF_options.verbose = true;
    EKF.KF_options.enable_profiler = true;
    
#ifdef SAVE_GT_LOGS
    CFileOutputStream  fo_log_ekf("log_GT_vs_EKF.txt");
    fo_log_ekf.printf("%%%% GT_X  GT_Y  EKF_MEAN_X  EKF_MEAN_Y   EKF_STD_X   EKF_STD_Y\n");
#endif
    
    // Init. simulation:
    // -------------------------
    float x=VEHICLE_INITIAL_X,y=VEHICLE_INITIAL_Y,phi=DEG2RAD(-180),v=VEHICLE_INITIAL_V,w=VEHICLE_INITIAL_W;
    float  t=0;
    
    while (winEKF.isOpen() && !mrpt::system::os::kbhit() )
    {
        // Update vehicle:
        x+=v*DELTA_TIME*(cos(phi)-sin(phi));
        y+=v*DELTA_TIME*(sin(phi)+cos(phi));
        phi+=w*DELTA_TIME;
        
        v+=1.0f*DELTA_TIME*cos(t);
        w-=0.1f*DELTA_TIME*sin(t);
        
        
        // Simulate noisy observation:
        float realBearing = atan2( y,x );
        float obsBearing = realBearing  + BEARING_SENSOR_NOISE_STD * randomGenerator.drawGaussian1D_normalized();
        printf("Real/Simulated bearing: %.03f / %.03f deg\n", RAD2DEG(realBearing), RAD2DEG(obsBearing) );
        
        float realRange = sqrt(square(x)+square(y));
        float obsRange = max(0.0, realRange  + RANGE_SENSOR_NOISE_STD * randomGenerator.drawGaussian1D_normalized() );
        printf("Real/Simulated range: %.03f / %.03f \n", realRange, obsRange );
        
        // Process with EKF:
        EKF.doProcess(DELTA_TIME,obsRange, obsBearing);
        
        EKF.getProfiler().enter("PF:complete_step");

        EKF.getProfiler().leave("PF:complete_step");
        
        // Show EKF state:
        CKFTracking::KFVector EKF_xkk;
        CKFTracking::KFMatrix EKF_pkk;
        
        EKF.getState( EKF_xkk, EKF_pkk );
        
        printf("Real: x:%.03f  y=%.03f heading=%.03f v=%.03f w=%.03f\n",x,y,phi,v,w);
        cout << "EKF: " << EKF_xkk << endl;
        
        // Draw EKF state:
        CKFTracking::KFMatrix   COVXY(2,2);
        COVXY(0,0) = EKF_pkk(0,0);
        COVXY(1,1) = EKF_pkk(1,1);
        COVXY(0,1) = COVXY(1,0) = EKF_pkk(0,1);
        
        winEKF.plotEllipse( EKF_xkk[0], EKF_xkk[1], COVXY, 3, "b-2", "ellipse_EKF" );
        
        // Save GT vs EKF state:
#ifdef SAVE_GT_LOGS
        // %% GT_X  GT_Y  EKF_MEAN_X  EKF_MEAN_Y   EKF_STD_X   EKF_STD_Y:
        fo_log_ekf.printf("%f %f %f %f %f %f\n",
                          x,y, // Real (GT)
                          EKF_xkk[0], EKF_xkk[1],
                std::sqrt(EKF_pkk(0,0)), std::sqrt(EKF_pkk(1,1))
                );
#endif
        
        // Draw the velocity vector:
        vector_float vx(2),vy(2);
        vx[0] = EKF_xkk[0];  vx[1] = vx[0] + EKF_xkk[2] * 1;
        vy[0] = EKF_xkk[1];  vy[1] = vy[0] + EKF_xkk[3] * 1;
        winEKF.plot( vx,vy, "g-4", "velocityEKF" );
        
        // Draw GT:
        winEKF.plot( vector_float(1,x), vector_float(1,y),"k.8","plot_GT");
        
        // Draw noisy observations:
        vector_float  obs_x(2),obs_y(2);
        obs_x[0] = obs_y[0] = 0;
        obs_x[1] = obsRange * cos( obsBearing );
        obs_y[1] = obsRange * sin( obsBearing );
        
        winEKF.plot(obs_x,obs_y,"r", "plot_obs_ray");
        
        // Delay:
        mrpt::system::sleep((int)(DELTA_TIME*1000));
        t+=DELTA_TIME;
    }
}


int main()
{
    try
    {
        TestBayesianTracking();
        return 0;
    } catch (std::exception &e)
    {
        std::cout << "MRPT exception caught: " << e.what() << std::endl;
        return -1;
    }
    catch (...)
    {
        printf("Untyped exception!!");
        return -1;
    }
}
