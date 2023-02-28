// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <ThreadPoolContainer.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <XMLFunctionsXsens.h>
#include <thread>
#include <future>
#include <functional>
#include <IMUHandler.h>
#include <cmath>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;



void manageIK() {
	OpenSimLive::IMUHandler genericDataReader;

	std::string manufacturerStr = ConfigReader("MainConfiguration.xml", "IMU_manufacturer");
	OpenSimLive::IMUHandler::IMUType manufacturer = OpenSimLive::IMUHandler::IMUType::simulated; // default to simulated in case the following if-statements fail
	if (manufacturerStr == "delsys")
		manufacturer = OpenSimLive::IMUHandler::IMUType::delsys;
	else if (manufacturerStr == "xsens")
		manufacturer = OpenSimLive::IMUHandler::IMUType::xsens;
	else if (manufacturerStr == "simulated")
		manufacturer = OpenSimLive::IMUHandler::IMUType::simulated;

	genericDataReader.setManufacturer(manufacturer);

	genericDataReader.initialize();


		
	//std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	bool enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false

	

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	IKTool.setReportErrors(saveIKResults);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	
	// CALIBRATION STEP
	// if we are not using simulated data, get data first; otherwise, initialization will have created a set of identity quaternions for calibration
	if (manufacturer != OpenSimLive::IMUHandler::IMUType::simulated) {
		genericDataReader.updateQuaternionTable();
	}
	// fill a timeseriestable with quaternion orientations of IMUs
	OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable = genericDataReader.getQuaternionTable();
	// calibrate the model and return its file name
	std::string calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), quaternionTimeSeriesTable);
	// give IKTool the necessary inputs and run it
	IKTool.setModelFile(calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(quaternionTimeSeriesTable); // the orientations of IMUs
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setTime(0); // set the time of the first state as 0 at calibration
	IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
	IKTool.run(false); // true for visualization
	std::cout << "Model has been calibrated." << std::endl;
	// set private variables to be accessed in IK calculations
	if (enableMirrorTherapy == true) {
		IKTool.setPointTrackerBodyName(ConfigReader("MainConfiguration.xml", "station_parent_body"));
		IKTool.setPointTrackerReferenceBodyName(ConfigReader("MainConfiguration.xml", "station_reference_body"));
		IKTool.setPointTrackerEnabled(true);
		IKTool.setSavePointTrackerResults(true);
	}
	else {
		IKTool.setPointTrackerEnabled(false);
	}


	std::cout << "Entering measurement loop." << std::endl;
	std::chrono::duration<double> clockDuration;
	std::chrono::steady_clock::time_point clockStart = std::chrono::high_resolution_clock::now();
	

	unsigned int iteration = 0;
	std::vector<double> IKTimes;
	do {
		OpenSim::TimeSeriesTableQuaternion quatTable = genericDataReader.updateAndGetQuaternionTable();
		clockDuration = (std::chrono::high_resolution_clock::now() - clockStart);
		double time = clockDuration.count();
		IKTool.updateOrdered(false, quatTable, ++iteration, time);
		clockDuration = (std::chrono::high_resolution_clock::now() - clockStart);
		double time2 = clockDuration.count();
		IKTimes.push_back(time2 - time);

	} while (iteration < 10000);



	clockDuration = std::chrono::high_resolution_clock::now() - clockStart;
	double finalTime = clockDuration.count();

	std::cout << "Performed " << iteration << " iterations in " << finalTime << " seconds." << std::endl;
	std::cout << "Frame rate: " << ((double)iteration / finalTime) << " iterations per second." << std::endl;

	double timeSum = 0;
	for (unsigned int k = 0; k < IKTimes.size(); ++k) {
		timeSum += IKTimes[k];
	}
	
	double meanDelay = timeSum / (double)iteration;

	double stdDiff = 0;
	for (unsigned int k = 0; k < IKTimes.size(); ++k) {
		stdDiff += (IKTimes[k] - meanDelay) * (IKTimes[k] - meanDelay);
	}
	double std = std::sqrt(stdDiff / (double)iteration);

	std::cout << "Mean delay: " << meanDelay*1000 << " ms, std: " << std*1000 << " ms" << std::endl;


	if (saveIKResults) {
		std::cout << "Saving IK results to file..." << std::endl;
		if (iteration < 100000) {
			IKTool.reportToFile();
		}
		else
		{
			std::cout << "More than 100 000 iterations calculated, as a safety precaution program is not saving results to file!" << std::endl;
		}
	}

	// close the connection to IMUs
	genericDataReader.closeConnection();

	return;
}


int main(int argc, char *argv[])
{

	// perform IK etc
	manageIK();

	std::cout << "Program finished." << std::endl;
	return 1;
}
