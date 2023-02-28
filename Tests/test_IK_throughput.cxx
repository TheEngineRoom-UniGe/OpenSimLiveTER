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

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;


// Holds various variables that are passed to several functions so that function arguments can be reduced by passing only this struct or a reference to it.
struct VariableManager {
	std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	bool enableMirrorTherapy = false;
	unsigned int maxThreads = stoi(ConfigReader("MainConfiguration.xml", "threads")); // get the maximum number of concurrent threads for multithreading
	std::string manufacturer = ConfigReader("MainConfiguration.xml", "IMU_manufacturer");

	std::chrono::steady_clock::time_point clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop


	unsigned int orderIndex = 0;
	std::queue<OpenSim::TimeSeriesTableQuaternion> orientationBuffer;
	std::queue<double> timeBuffer;
	bool bufferInUse = false;
	unsigned int maxBufferSize = 8;
	bool trialDone = false;
	std::atomic<bool> runProducerThread = true;
	std::atomic<bool> runIKThread = true;
	double trialDuration = 0;

}; // struct dataHolder ends



void updateConcurrentIKTool(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, VariableManager& vm, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, double time, unsigned int orderIndex) {
	IKTool.updateOrdered(false, quatTable, orderIndex, time);
}



std::mutex mainMutex;

// This function reads quaternion values in a loop and saves them in a queue buffer.
void producerThread(OpenSimLive::IMUHandler& genericDataReader, VariableManager& vm) {
	
	
	// update new quaternion table but don't get it yet; do this ONLY ONCE to do all the IK with the same orientations
	genericDataReader.updateQuaternionTable();
	
	do {

		// get time stamp
		std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - vm.clockStart;
		double time = duration.count();

		

		// if consumer is not accessing the buffer and the buffer size is not maximal
		if (!vm.bufferInUse && vm.orientationBuffer.size() < vm.maxBufferSize)
		{
			// set bufferInUse to true to prevent consumer from accessing the buffer
			vm.bufferInUse = true;
			std::lock_guard<std::mutex> lock(mainMutex);
			// push time stamp to the shared buffer
			vm.timeBuffer.push(time);
			// push the time series table to the shared buffer
			vm.orientationBuffer.push(genericDataReader.getQuaternionTable());
			vm.bufferInUse = false;
		}
		if (time > vm.trialDuration) {
			vm.runIKThread = false;
			vm.runProducerThread = false;
		}

	} while (vm.runProducerThread);

	std::cout << "Producer done!" << std::endl;
}

// This function reads quaternion tables from a queue buffer and performs IK on them.
void consumerThread(VariableManager& vm, OpenSimLive::IMUInverseKinematicsToolLive& IKTool, OpenSimLive::ThreadPoolContainer& threadPoolContainer) {

	do {
		// if producer is not accessing the buffer and the buffer contains values
		if (!vm.bufferInUse && vm.orientationBuffer.size() > 0) {
			vm.bufferInUse = true;
			std::lock_guard<std::mutex> lock(mainMutex);
			// get and pop time from the buffer
			double time = vm.timeBuffer.front();
			vm.timeBuffer.pop();
			// get and pop the front of the queue
			OpenSim::TimeSeriesTableQuaternion quatTable(vm.orientationBuffer.front());
			vm.orientationBuffer.pop();
			vm.bufferInUse = false;
			// start a new thread where the IK is calculated and increment the number of IK operations done
			threadPoolContainer.offerFuture(updateConcurrentIKTool, std::ref(IKTool), std::ref(vm), quatTable, time, ++vm.orderIndex);
		}

	} while (vm.runIKThread);
	std::cout << "Consumer done!" << std::endl;
}








void manageIK(double inputSeconds, int inputThreads) {

	bool saveQuaternions = (ConfigReader("MainConfiguration.xml", "save_quaternions_to_file") == "true");
	

	VariableManager vm;

	// create Xsens connection object and connect the program to IMUs
	OpenSimLive::IMUHandler genericDataReader;

	OpenSimLive::IMUHandler::IMUType manufacturer = OpenSimLive::IMUHandler::IMUType::simulated; // default to simulated if the if-statement below fails
	if (vm.manufacturer == "delsys")
		manufacturer = OpenSimLive::IMUHandler::IMUType::delsys;
	else if (vm.manufacturer == "xsens")
		manufacturer = OpenSimLive::IMUHandler::IMUType::xsens;
	else if (vm.manufacturer == "simulated")
		manufacturer = OpenSimLive::IMUHandler::IMUType::simulated;
	// set the value in the IMUHandler object
	genericDataReader.setManufacturer(manufacturer);
	// establish connection to IMUs
	genericDataReader.initialize();

	vm.trialDuration = inputSeconds;
		
	//std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	vm.saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	vm.enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false

	

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	IKTool.setReportErrors(vm.saveIKResults);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	
	// CALIBRATION STEP
	// fill a timeseriestable with quaternion orientations of IMUs
	OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable = genericDataReader.getQuaternionTable();
	// calibrate the model and return its file name
	vm.calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), quaternionTimeSeriesTable);
	// give IKTool the necessary inputs and run it
	IKTool.setModelFile(vm.calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(quaternionTimeSeriesTable); // the orientations of IMUs
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setTime(0); // set the time of the first state as 0 at calibration
	IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
	IKTool.run(false); // true for visualization
	std::cout << "Model has been calibrated." << std::endl;
	// set private variables to be accessed in IK calculations
	if (vm.enableMirrorTherapy == true) {
		IKTool.setPointTrackerBodyName(ConfigReader("MainConfiguration.xml", "station_parent_body"));
		IKTool.setPointTrackerReferenceBodyName(ConfigReader("MainConfiguration.xml", "station_reference_body"));
		IKTool.setPointTrackerEnabled(true);
		IKTool.setSavePointTrackerResults(true);
	}
	else {
		IKTool.setPointTrackerEnabled(false);
	}

	unsigned int maxThreads = inputThreads;

	OpenSimLive::ThreadPoolContainer threadPoolContainer(maxThreads);

	std::cout << "Entering measurement loop." << std::endl;

	// reset clock
	vm.clockStart = std::chrono::high_resolution_clock::now();

	std::thread producer(producerThread, std::ref(genericDataReader), std::ref(vm));
	std::thread consumer(consumerThread, std::ref(vm), std::ref(IKTool), std::ref(threadPoolContainer));

	producer.join();
	consumer.join();

	// wait until all IK threads are finished, otherwise reporting IK to file may throw an exception
	threadPoolContainer.waitForFinish();

	std::chrono::duration<double> clockDuration = std::chrono::high_resolution_clock::now() - vm.clockStart;
	double finalTime = clockDuration.count();

	std::cout << "Performed " << vm.orderIndex << " iterations in " << finalTime << " seconds." << std::endl;
	std::cout << "Frame rate: " << ((double)vm.orderIndex / finalTime) << " iterations per second." << std::endl;

	if (vm.saveIKResults) {
		std::cout << "Saving IK results to file..." << std::endl;
		if (vm.orderIndex < 100000) {
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
	std::string inputSecsStr;
	std::cout << "Please input test duration in seconds: ";
	std::cin >> inputSecsStr;
	double inputSecs = stod(inputSecsStr);

	std::string inputThreadsStr;
	std::cout << "Please input the number of threads to be used: ";
	std::cin >> inputThreadsStr;
	int inputThreads = stoi(inputThreadsStr);

	// perform IK etc
	manageIK(inputSecs, inputThreads);

	std::cout << "Program finished." << std::endl;
	return 1;
}
