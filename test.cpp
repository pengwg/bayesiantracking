#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <QDateTime>

#include "KFTracking.h"

using namespace mrpt::gui;
using namespace mrpt::random;
using namespace std;

//#define SAVE_GT_LOGS

QDateTime getRecordTime(std::ifstream &inFile, float &X, float &Y, float &Z)
{
    char record[100];
    inFile.getline(record, 100);
    std::istringstream stream(&record[0]);
    char date[100];
    stream >> date >> X >> Y >> Z;

    return QDateTime::fromString(date, "yyyy-MM-dd_hh:mm:ss.zzz");
}

void TestBayesianTracking()
{
    randomGenerator.randomize();
    
    CDisplayWindowPlots		winEKF("Tracking - Extended Kalman Filter",900,800);

    // Create EKF
    // ----------------------
    // Init. simulation:
    // -------------------------
    std::ifstream trackFile("../data/2012-07-24_14:49:51.449.tracking1");
    float x, y, z;
    QDateTime t = getRecordTime(trackFile, x, y, z);

    CKFTracking	EKF(x, z);
    //EKF.KF_options.method = kfEKFNaive;
    
    EKF.KF_options.verbose = true;
    EKF.KF_options.enable_profiler = true;
    
#ifdef SAVE_GT_LOGS
    CFileOutputStream  fo_log_ekf("log_GT_vs_EKF.txt");
    fo_log_ekf.printf("%%%% GT_X  GT_Y  EKF_MEAN_X  EKF_MEAN_Y   EKF_STD_X   EKF_STD_Y\n");
#endif
    
    winEKF.setPos(10,10);
    winEKF.axis((int)x-20, 2, (int)z-20, 2);
    //winEKF.axis(-2,20,-10,10);
    winEKF.axis_equal();
    
    while (winEKF.isOpen() && !mrpt::system::os::kbhit() )
    {
        QDateTime t0 = t;
        t = getRecordTime(trackFile, x, y, z);

        float delta_time = t0.msecsTo(t) / 1000.0;

        cout << "Record: "
             << delta_time << "s, "
             << x << "mm, "
             << y << "mm, "
             << z << "mm" << endl;

        // Process with EKF:
        EKF.doProcess(delta_time, x, z);
        
        // Show EKF state:
        CKFTracking::KFVector EKF_xkk;
        CKFTracking::KFMatrix EKF_pkk;
        
        EKF.getState( EKF_xkk, EKF_pkk );
        
        cout << "EKF: " << EKF_xkk << endl;
        
        // Draw EKF state:
        float theta = EKF_xkk[0];
        float a = EKF_xkk[2];
        float b = EKF_xkk[3];
        float xc = EKF_xkk[4];
        float yc = EKF_xkk[5];

        float x1 = xc + a * cosf(theta);
        float y1 = yc + b * sinf(theta);

        winEKF.plot(vector_float(1, x1), vector_float(1, y1), "b.8", "EKF Estimate" );

        
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
        vector_float rx(2), ry(2);
        rx[0] = xc - a;  rx[1] = xc + a;
        ry[0] = yc;  ry[1] = yc;
        winEKF.plot( rx, ry, "g-4", "R1" );
        
        rx[0] = xc;  rx[1] = xc;
        ry[0] = yc - b;  ry[1] = yc + b;
        winEKF.plot( rx, ry, "r-4", "R2" );

        // Draw GT:
        winEKF.plot( vector_float(1,x), vector_float(1,z),"k.8","plot_GT");
        
        // Delay:
        mrpt::system::sleep(1000);
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
