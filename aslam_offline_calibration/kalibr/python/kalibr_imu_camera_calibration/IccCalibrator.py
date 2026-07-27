import aslam_backend as aopt
import aslam_splines as asp
from . import IccUtil as util
import incremental_calibration as inc
import kalibr_common as kc
import sm

import gc
import yaml
import numpy as np
import multiprocessing
import sys

# make numpy print prettier
np.set_printoptions(suppress=True)

CALIBRATION_GROUP_ID = 0
HELPER_GROUP_ID = 1

def addSplineDesignVariables(problem, dvc, setActive=True, group_id=HELPER_GROUP_ID):
    for i in range(0,dvc.numDesignVariables()):
        dv = dvc.designVariable(i)
        dv.setActive(setActive)
        problem.addDesignVariable(dv, group_id)

class IccCalibrator(object):
    def __init__(self):
        self.ImuList = []
        self.DvlList = []

    def initDesignVariables(self, problem, poseSpline, noTimeCalibration, noChainExtrinsics=True, \
                            estimateGravityLength=False, initialGravityEstimate=np.array([0.0,9.81,0.0])):        
        # Initialize the system pose spline (always attached to imu0) 
        self.poseDv = asp.BSplinePoseDesignVariable( poseSpline )
        addSplineDesignVariables(problem, self.poseDv)

        # Add the calibration target orientation design variable. (expressed as gravity vector in target frame)
        if estimateGravityLength:
            self.gravityDv = aopt.EuclideanPointDv( initialGravityEstimate )
        else:
            self.gravityDv = aopt.EuclideanDirection( initialGravityEstimate )
        self.gravityExpression = self.gravityDv.toExpression()  
        self.gravityDv.setActive( True )
        problem.addDesignVariable(self.gravityDv, HELPER_GROUP_ID)
        
        #Add all DVs for all IMUs
        for imu in self.ImuList:
            imu.addDesignVariables( problem )

        #Add all DVs for all DVLs (extrinseco, escala)
        for dvl in self.DvlList:
            dvl.addDesignVariables( problem )

        #Add all DVs for the camera chain
        self.CameraChain.addDesignVariables( problem, noTimeCalibration, noChainExtrinsics )

    def addPoseMotionTerms(self, problem, tv, rv):
        wt = 1.0/tv;
        wr = 1.0/rv
        W = np.diag([wt,wt,wt,wr,wr,wr])
        asp.addMotionErrorTerms(problem, self.poseDv, W, errorOrder)
        
    #add camera to sensor list (create list if necessary)
    def registerCamChain(self, sensor):
        self.CameraChain = sensor

    def registerImu(self, sensor):
        self.ImuList.append( sensor )

    def registerDvl(self, sensor):
        self.DvlList.append( sensor )
            
    def buildProblem( self, 
                      splineOrder=6, 
                      poseKnotsPerSecond=70, 
                      biasKnotsPerSecond=70, 
                      doPoseMotionError=False, 
                      mrTranslationVariance=1e6,
                      mrRotationVariance=1e5,
                      doBiasMotionError=True,
                      blakeZisserCam=-1,
                      huberAccel=-1,
                      huberGyro=-1,
                      noTimeCalibration=False,
                      noChainExtrinsics=True,
                      maxIterations=20,
                      gyroNoiseScale=1.0,
                      accelNoiseScale=1.0,
                      timeOffsetPadding=0.02,
                      recompute_cam_imu=False,
                      huberDvl=-1,
                      verbose=False  ):

        print("\tSpline order: %d" % (splineOrder))
        print("\tPose knots per second: %d" % (poseKnotsPerSecond))
        print("\tDo pose motion regularization: %s" % (doPoseMotionError))
        print("\t\txddot translation variance: %f" % (mrTranslationVariance))
        print("\t\txddot rotation variance: %f" % (mrRotationVariance))
        print("\tBias knots per second: %d" % (biasKnotsPerSecond))
        print("\tDo bias motion regularization: %s" % (doBiasMotionError))
        print("\tBlake-Zisserman on reprojection errors %s" % blakeZisserCam)
        print("\tAcceleration Huber width (sigma): %f" % (huberAccel))
        print("\tGyroscope Huber width (sigma): %f" % (huberGyro))
        print("\tDo time calibration: %s" % (not noTimeCalibration))
        print("\tMax iterations: %d" % (maxIterations))
        print("\tTime offset padding: %f" % (timeOffsetPadding))


        ############################################
        ## initialize camera chain
        ############################################
        #estimate the timeshift for all cameras to the main imu
        self.noTimeCalibration = noTimeCalibration
        if not noTimeCalibration:
            for cam in self.CameraChain.camList:
                cam.findTimeshiftCameraImuPrior(self.ImuList[0], verbose)
        
        #obtain orientation prior between main imu and camera chain (if no external input provided)
        #and initial estimate for the direction of gravity
        self.CameraChain.findOrientationPriorCameraChainToImu(self.ImuList[0])
        estimatedGravity = self.CameraChain.getEstimatedGravity()

        # Mode A (DVL): reuse the PROVIDED camera-IMU calibration (--cams com T_cam_imu) como valor inicial
        # do extrinseco cam0, sobrescrevendo a re-estimativa do orientation-prior (que so estima rotacao,
        # deixando a translacao/lever-arm em 0). Depois esse valor e fixado por fixCamImuDesignVariables. (D12)
        if self.DvlList and not recompute_cam_imu:
            self.reuseProvidedCamImuExtrinsics()

        ############################################
        ## init optimization problem
        ############################################
        #initialize a pose spline using the camera poses in the camera chain
        poseSpline = self.CameraChain.initializePoseSplineFromCameraChain(splineOrder, poseKnotsPerSecond, timeOffsetPadding)
        
        # Initialize bias splines for all IMUs
        for imu in self.ImuList:
            imu.initBiasSplines(poseSpline, splineOrder, biasKnotsPerSecond)
        
        # Now I can build the problem
        problem = inc.CalibrationOptimizationProblem()

        # Initialize all design variables.
        self.initDesignVariables(problem, poseSpline, noTimeCalibration, noChainExtrinsics, initialGravityEstimate = estimatedGravity)
        
        ############################################
        ## add error terms
        ############################################
        #Add calibration target reprojection error terms for all camera in chain
        self.CameraChain.addCameraChainErrorTerms(problem, self.poseDv, blakeZissermanDf=blakeZisserCam, timeOffsetPadding=timeOffsetPadding)
        
        # Initialize IMU error terms.
        for imu in self.ImuList:
            imu.addAccelerometerErrorTerms(problem, self.poseDv, self.gravityExpression, mSigma=huberAccel, accelNoiseScale=accelNoiseScale)
            imu.addGyroscopeErrorTerms(problem, self.poseDv, mSigma=huberGyro, gyroNoiseScale=gyroNoiseScale, g_w=self.gravityExpression)

            # Add the bias motion terms.
            if doBiasMotionError:
                imu.addBiasMotionTerms(problem)

        # Initialize DVL error terms (velocity residuals) + temporal offset prior.
        for dvl in self.DvlList:
            dvl.findTimeOffsetPrior(self.poseDv)
            dvl.addVelocityErrorTerms(problem, self.poseDv,
                                      huber=(huberDvl if huberDvl > 0 else 0.0))

        # Mode A (default) for DVL calibration: hold the reused camera-IMU calibration FIXED, so only
        # the trajectory spline, IMU biases, gravity and the DVL parameters are optimized. Mode B
        # (recompute_cam_imu=True) leaves the camera-IMU design variables active (joint optimization).
        if self.DvlList and not recompute_cam_imu:
            self.fixCamImuDesignVariables()

        # Add the pose motion terms.
        if doPoseMotionError:
            self.addPoseMotionTerms(problem, mrTranslationVariance, mrRotationVariance)
        
        # Add a gravity prior
        self.problem = problem


    def optimize(self, options=None, maxIterations=30, recoverCov=False):

        if options is None:
            options = aopt.Optimizer2Options()
            options.verbose = True
            options.doLevenbergMarquardt = True
            options.levenbergMarquardtLambdaInit = 10.0
            options.nThreads = max(1,multiprocessing.cpu_count()-1)
            options.convergenceDeltaX = 1e-5
            options.convergenceDeltaJ = 1e-2
            options.maxIterations = maxIterations
            options.trustRegionPolicy = aopt.LevenbergMarquardtTrustRegionPolicy(options.levenbergMarquardtLambdaInit)
            options.linearSolver = aopt.BlockCholeskyLinearSystemSolver() #does not have multi-threading support

        #run the optimization
        self.optimizer = aopt.Optimizer2(options)
        self.optimizer.setProblem(self.problem)

        optimizationFailed=False
        try:
            retval = self.optimizer.optimize()
            if retval.linearSolverFailure:
                optimizationFailed = True
        except Exception as e:
            sm.logError(str(e))
            optimizationFailed = True

        if optimizationFailed:
            sm.logError("Optimization failed!")
            raise RuntimeError("Optimization failed!")
        
        #free some memory
        del self.optimizer
        gc.collect()
        if recoverCov:
            self.recoverCovariance()
        

    def recoverCovariance(self):
        #Covariance ordering (=dv ordering)
        #ORDERING:   N=num cams
        #            1. transformation imu-cam0 --> 6
        #            2. camera time2imu --> 1*numCams (only if enabled)
        
        print("Recovering covariance...")
        estimator = inc.IncrementalEstimator(CALIBRATION_GROUP_ID)
        rval = estimator.addBatch(self.problem, True)    
        est_stds = np.sqrt(estimator.getSigma2Theta().diagonal())
        
        #split and store the variance
        self.std_trafo_ic = np.array(est_stds[0:6])
        self.std_times = np.array(est_stds[6:])
    
    def reuseProvidedCamImuExtrinsics(self):
        """Modo A: inicializa o extrinseco cam0 com o T_cam_imu fornecido em --cams (D12).

        O fluxo padrao do Kalibr comeca com T_extrinsic=identidade e so estima a rotacao (translacao 0);
        fixar isso no Modo A daria uma translacao errada. Aqui usamos o T_cam_imu da calibracao cam-IMU
        submersa ja validada (camchain-imucam.yaml). So cam0 (imu->cam0); baselines cam-cam ja vem do config.
        """
        chainConfig = self.CameraChain.chainConfig
        try:
            T_cam0_imu = chainConfig.getExtrinsicsImuToCam(0)
            self.CameraChain.camList[0].T_extrinsic = T_cam0_imu
            print("Mode A: reusing provided T_cam_imu (cam0) as the fixed camera-IMU extrinsic.")
        except Exception:
            sm.logWarn("Mode A: --cams has no T_cam_imu (cam0); using the estimated orientation prior "
                       "(translation may be 0/inaccurate). Pass a camchain-imucam.yaml from "
                       "kalibr_calibrate_imu_camera for a correct reuse.")

    def fixCamImuDesignVariables(self):
        """Modo A: desativa os design variables da calibracao camera-IMU (reusada como fixa).

        Mantem ativos: a spline de pose (trajetoria), os biases da IMU, a gravidade e os DVs do DVL.
        Desativa: extrinseco imu->cam (T_c_b), timeshift camera-IMU, e extrinsecos imu-imu (q_i_b, r_b).
        """
        for cam in self.CameraChain.camList:
            for i in range(0, cam.T_c_b_Dv.numDesignVariables()):
                cam.T_c_b_Dv.getDesignVariable(i).setActive(False)
            cam.cameraTimeToImuTimeDv.setActive(False)
        for imu in self.ImuList:
            imu.q_i_b_Dv.setActive(False)
            imu.r_b_Dv.setActive(False)
        print("Mode A: camera-IMU calibration held fixed (reused); only trajectory/biases/DVL are free.")

    def saveDvlParametersYaml(self, resultFile):
        """Escreve o resultado da calibracao do DVL (T_dvl_imu SE3, escala, timeshift) em YAML."""
        results = {}
        for dvlNr, dvl in enumerate(self.DvlList):
            results["dvl{0}".format(dvlNr)] = {
                "T_dvl_imu": np.array(dvl.getResultTransformation()).tolist(),
                "velocity_scale": float(dvl.getResultScale()),
                "timeshift_dvl_imu": float(dvl.getResultTimeOffset()),
                "sound_speed": dvl.dvlConfig.getSoundSpeed(),
            }
        with open(resultFile, "w") as outfile:
            outfile.write(yaml.dump(results, default_flow_style=None, width=2147483647))

    def saveDvlResultTxt(self, filename):
        """Resumo textual da calibracao do DVL: extrinseco, escala, offset e estatisticas de residuo."""
        with open(filename, "w") as f:
            f.write("Calibration results (DVL)\n")
            f.write("=========================\n")
            for dvlNr, dvl in enumerate(self.DvlList):
                T = np.array(dvl.getResultTransformation())
                st = dvl.getResidualStats()
                f.write("\nDVL{0}\n".format(dvlNr))
                f.write("  Reference frame: IMU (reference)\n")
                f.write("  T_dvl_imu (SE3, IMU -> DVL):\n")
                f.write("{0}\n".format(T))
                f.write("  translation (lever-arm proj.) [m]: {0}\n".format(T[:3, 3]))
                f.write("  velocity_scale (sound-speed): {0}\n".format(dvl.getResultScale()))
                f.write("  timeshift_dvl_imu [s]: {0}\n".format(dvl.getResultTimeOffset()))
                f.write("  --- velocity residuals ---\n")
                f.write("  RMS [m/s]: {0}\n".format(st["rms"]))
                f.write("  per-axis RMS [m/s]: {0}\n".format(st["per_axis_rms"]))
                f.write("  mean / median / max norm [m/s]: {0} / {1} / {2}\n".format(
                    st["mean_norm"], st["median_norm"], st["max_norm"]))
                f.write("  measurements used / total (skipped by gating/bounds): {0} / {1} ({2})\n".format(
                    st["n_used"], st["n_total"], st["n_skipped"]))

    def saveImuSetParametersYaml(self, resultFile):
        imuSetConfig = kc.ImuSetParameters(resultFile, True)
        for imu in self.ImuList:
            imuConfig = imu.getImuConfig()
            imuSetConfig.addImuParameters(imu_parameters=imuConfig)
        imuSetConfig.writeYaml(resultFile)

    def saveCamChainParametersYaml(self, resultFile):    
        chain = self.CameraChain.chainConfig
        nCams = len(self.CameraChain.camList)
    
        # Calibration results
        for camNr in range(0,nCams):
            #cam-cam baselines           
            if camNr > 0:
                T_cB_cA, baseline = self.CameraChain.getResultBaseline(camNr-1, camNr)
                chain.setExtrinsicsLastCamToHere(camNr, T_cB_cA)

            #imu-cam trafos
            T_ci = self.CameraChain.getResultTrafoImuToCam(camNr)
            chain.setExtrinsicsImuToCam(camNr, T_ci)

            if not self.noTimeCalibration:
                #imu to cam timeshift
                timeshift = float(self.CameraChain.getResultTimeShift(camNr))
                chain.setTimeshiftCamImu(camNr, timeshift)
             
        try:
            chain.writeYaml(resultFile)
        except:
            raise RuntimeError("ERROR: Could not write parameters to file: {0}\n".format(resultFile))
