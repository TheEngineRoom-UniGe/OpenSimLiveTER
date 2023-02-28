// main.cpp

#include <MpuDataReader.h>
#include <string>
#include <iostream>
#include <OpenSim.h>
#include <XMLFunctions.h>
#include <sstream>
#define BUFFSIZE 35

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

using namespace OpenSimLive;

// CONSTRUCTOR
MpuDataReader::MpuDataReader() {
	//populateLabelVector();
	populateDictionary();
	// randomize seed based on current time
	srand(std::time(NULL));
}

// DESTRUCTOR
MpuDataReader::~MpuDataReader() {
	// if the setting is true, save quaternions to file
	if (saveQuaternionsToFile_) {
		saveQuaternionsToFile(OPENSIMLIVE_ROOT, "OpenSimLive-results");
	}
}
//create socket and wait
unsigned int MpuDataReader::InitiateStartupPhase() {
	int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (result < 0) {
		std::cerr << "WSA startup failed!" << std::endl;
		return -1;
	}

	this -> sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (this->sock == INVALID_SOCKET) {
		std::cerr << "Failed to create the socket!!!" << std::endl;
		WSACleanup();
		return -1;
	}

	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(2395);
	addr.sin_addr.S_un.S_addr = inet_addr("130.251.13.139");

	if (bind(this->sock, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR) {
		std::cerr << "Failed to bind the socket: " << WSAGetLastError() << std::endl;
		closesocket(sock);
		WSACleanup();
		return -1;
	}

	std::cerr << "Socket created! " << std::endl;
	std::cerr << "Listening to: " << "130.251.13.139" << std::endl;
	std::cerr << "On port: " << 2395 << std::endl;

	char buffer[BUFFSIZE];

	auto start = std::chrono::steady_clock::now();
	auto end = start + std::chrono::seconds(1);

	std::cout << "Listening for messages..." << std::endl;

	while (std::chrono::steady_clock::now() < end) {
		Message msg = ReadNewMsg(buffer);

		//if (true && id_to_joint[msg.id] == "MC3_imu") {
		if (false) {
			std::cout << "Message Received!!" << std::endl;
			std::cout << "Msg ID: " << id_to_joint[msg.id]  << std::endl;
			std::cout << "Quaternion: " << "\n\t" <<
				msg.quaternion[0] << "\n\t" <<
				msg.quaternion[1] << "\n\t" <<
				msg.quaternion[2] << "\n\t" <<
				msg.quaternion[3] << "\n\t" <<
				std::endl;
		}

		//populates the label array
		if (std::find(labels_.begin(), labels_.end(), id_to_joint[msg.id]) == labels_.end()) {
			labels_.push_back(id_to_joint[msg.id]);
			labelsSize_ = labels_.size();
			calibration_.push_back(msg);
		}

	}
	std::cout << "Populated the label array" << std::endl;
	return 0;
}

Message MpuDataReader::ReadNewMsg(char buffer[]) {
	memset(buffer, 0, BUFFSIZE);
	int bytesReceived = recv(this->sock, buffer, BUFFSIZE, 0);
	if (!(bytesReceived == BUFFSIZE)) {
		std::cerr << "Packet size is different from the expected one!" << std::endl;
	}
	std::stringstream ss;
	ss.write(buffer, bytesReceived);

	Message msg;


	memcpy(msg.quaternion, buffer, sizeof(float) * 4);
	memcpy(msg.accelerometer, buffer + sizeof(float) * 4, sizeof(short) * 3);
	memcpy(msg.gyroscope, buffer + sizeof(float) * 4 + sizeof(short) * 3, sizeof(short) * 3);
	memcpy(msg.magnetometer, buffer + sizeof(float) * 4 + sizeof(short) * 6, sizeof(short) * 3);
	memcpy(&msg.id, buffer + sizeof(float) * 4 + sizeof(short) * 9, sizeof(char));


	return msg;
}


void MpuDataReader::populateDictionary() {
	id_to_joint[0] = "MC1_imu";
	id_to_joint[1] = "PD1_imu";
	id_to_joint[10] = "PP2_imu";
	id_to_joint[11] = "PM2_imu";
	id_to_joint[20] = "PP4_imu";
	id_to_joint[21] = "PM4_imu";
	id_to_joint[30] = "PP3_imu";
	id_to_joint[31] = "PM3_imu";
	id_to_joint[100] = "wrist_imu";
	id_to_joint[101] = "wrist_imu";
	id_to_joint[110] = "PP5_imu";
	id_to_joint[111] = "PM5_imu";
	id_to_joint[120] = "MC3_imu";
	id_to_joint[121] = "wrist_imu";
	id_to_joint[130] = "MC1_imu";
	id_to_joint[131] = "PD1_imu";

}

// creates a quaternion with randomized elements and returns it as a unit quaternion
SimTK::Quaternion MpuDataReader::generateQuaternion() {
	// create a quaternion (not a unit quaternion) with random elements
	SimTK::Quaternion quat(rand() % 100, rand() % 100, rand() % 100, rand() % 100);
	// return a normalized version of the random quaternion
	return quat.normalize();
}

void MpuDataReader::PrintLabels() {
	std::cout << "Total number of labels: " << labelsSize_ << std::endl;
	for (int i = 0; i < labelsSize_; i++) {
		std::cout << labels_[i] << std::endl;
	}
}

void  MpuDataReader::ResizeQuatVector() {
	// resize quaternions_ to equal the number of sensors aka the number of quaternions that will be saved in quaternions_
	quaternions_.resize(labelsSize_);
	// pre-reserve a fair amount of space in quaternionData_ to speed things up later
	quaternionData_.reserve(100000);
}

// populate labels_ from config file
void MpuDataReader::populateLabelVector() {
	labels_ = ConfigReaderVector("MainConfiguration.xml", "simulated_bodies");
	labelsSize_ = labels_.size();
	for (int i = 0; i < labelsSize_; i++) {
		std::cout << labels_[i] << std::endl;
	}
	// resize quaternions_ to equal the number of sensors aka the number of quaternions that will be saved in quaternions_
	quaternions_.resize(labelsSize_);
	// pre-reserve a fair amount of space in quaternionData_ to speed things up later
	quaternionData_.reserve(100000);
}


// updates quatTable_ with new quaternion values that are all identity quaternions; used for performance testing purposes
void MpuDataReader::generateIdentityQuaternions() {
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labelsSize_);
	std::cout << "generate quaternion matrix" << std::endl;

	// generate a new identity quaternion on a spot in the matrix
	SimTK::Quaternion quat(1, 0, 0, 0);
	std::cout << "generate identity" << std::endl;
	// loop through all elements of labels_ (the vector containing labels for IMUs on the model)

	for (unsigned int m = 0; m < labelsSize_; ++m) {
		quatMatrix.set(0, m, quat);
		if (saveQuaternionsToFile_) {
			//std::cout << quat << std::endl;
			quaternions_[m] = quat;
		}
	}

	updateTime();

	// finally create/overwrite the time series table using the identity quaternion data
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>({ 0 }, quatMatrix, labels_);
}

void MpuDataReader::generateCalibrationQuaternion() {
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labelsSize_);
	std::cout << "generate quaternion matrix" << std::endl;
	//char buffer[BUFFSIZE];
	//Message msg = this->ReadNewMsg(buffer);
	//// generate a new identity quaternion on a spot in the matrix
	//simtk::quaternion quat( msg.quaternion[3],
	//						msg.quaternion[0],
	//						msg.quaternion[1],
	//						msg.quaternion[2]);
	std::cout << "calibration samples: " << std::endl;
	std::cout << calibration_.size() << std::endl;


	//std::cout << quat << std::endl;

	// loop through all elements of labels_ (the vector containing labels for IMUs on the model)
	for (unsigned int m = 0; m < labelsSize_; ++m) {
		SimTK::Quaternion quat(calibration_[m].quaternion[3],
			calibration_[m].quaternion[0],
			calibration_[m].quaternion[1],
			calibration_[m].quaternion[2]);
		quatMatrix.set(0, m, quat);
		if (saveQuaternionsToFile_) {
			//std::cout << quat << std::endl;
			quaternions_[m] = quat;
		}
	}

	updateTime();

	// finally create/overwrite the time series table using the identity quaternion data
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>({ 0 }, quatMatrix, labels_);
}

OpenSim::TimeSeriesTable_<SimTK::Quaternion>  MpuDataReader::generateNewQuaternion() {
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labelsSize_);
	//std::cout << "generate quaternion matrix" << std::endl;

	char buffer[BUFFSIZE];
	int msg_counter = 0;

	while (msg_counter<11){
		// loop to read 11 messages before submitting the quaternion table to the following part of the code
		// could be changed to a "time-based approach" to set a fixed frequency
		Message msg = this->ReadNewMsg(buffer);

		auto link_name = this->id_to_joint[msg.id];
		// generate a new identity quaternion on a spot in the matrix
	

		//std::cout << "calibration samples: " << std::endl;
		//std::cout << calibration_.size() << std::endl;
		int found_idx = -1;
		for (int i = 0; i < labelsSize_; i++) {
			if (labels_[i] == link_name) {
				//std::cout << i << std::endl;
				found_idx = i;
				break;
			}
		}
		//std::cout << std::endl << std::endl << std::endl;
		//std::cout << quat << std::endl;
		calibration_[found_idx] = msg;
		// loop through all elements of labels_ (the vector containing labels for IMUs on the model)
		for (unsigned int m = 0; m < labelsSize_; ++m) {
			SimTK::Quaternion quat(	calibration_[m].quaternion[3],
									calibration_[m].quaternion[0],
									calibration_[m].quaternion[1],
									calibration_[m].quaternion[2]);
			//if (m == 0) {
			//	std::cout << quat << std::endl;
			//	std::cout << link_name << std::endl;
			//	}
				quatMatrix.set(0, m, quat);
			if (saveQuaternionsToFile_) {
				//std::cout << quat << std::endl;
				quaternions_[m] = quat;
			}
		}
		msg_counter++;
	}
	updateTime();

	// finally create/overwrite the time series table using the identity quaternion data
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>({ 0 }, quatMatrix, labels_);
	//std::cout << quatTable_ << std::endl;
	return quatTable_;
}


// update quat table with random unit quaternions
void MpuDataReader::generateRandomQuaternions() {
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labelsSize_);
	// loop through all elements of labels_ (the vector containing labels for IMUs on the model)
	for (unsigned int m = 0; m < labelsSize_; ++m) {
		// generate a new unit quaternion on a spot in the matrix
		SimTK::Quaternion quat = generateQuaternion();
		quatMatrix.set(0, m, quat);
		if (saveQuaternionsToFile_) {
			quaternions_[m] = quat;
		}
	}

	updateTime();

	// finally create/overwrite the time series table using the randomized quaternion data
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>({ 0 }, quatMatrix, labels_);
}


// updates quatTable_ with new quaternion values that are either identity quaternions or randomized unit quaternions
void MpuDataReader::updateQuaternionTable() {

	if (is_random_) {
		generateRandomQuaternions();
	}
	else {
		generateIdentityQuaternions();
	}
}


// this is actually pointless, because there is no actual connection to IMUs when simulating IMU data
void MpuDataReader::closeConnection() {
	std::cout << "Simulated connection closed!" << std::endl;
}


// This function saves the time points and the corresponding quaternions to file for later examination.
void MpuDataReader::saveQuaternionsToFile(const std::string& rootDir, const std::string& resultsDir) {
	
	if (timeVector_.size() > 100000 || quaternionData_.size() > 100000) {
		std::cout << "In a normal situation we would save quaternions to file now, but because there are " << timeVector_.size() << " data points, for the sake of hard drive space we won't do it." << std::endl;
		return;
	}
	else {
		std::cout << "Saving quaternion time series to file..." << std::endl;
	}

	// create the complete path of the file, including the file itself, as a string
	std::string filePath(rootDir + "/" + resultsDir + "/" + "QuaternionTimeSeriesSimulated.txt");
	// create an output file stream that is used to write into the file
	std::ofstream outputFile;
	// open and set file to discard any contents that existed in the file previously (truncate mode)
	outputFile.open(filePath, std::ios_base::out | std::ios_base::trunc);
	// check that the file was successfully opened and write into it
	if (outputFile.is_open())
	{
		outputFile << "Time series of measured orientation data in quaternions:\n";
		outputFile << "Time (s)";

		for (unsigned int j = 0; j < labelsSize_; ++j) {
			outputFile << "\t" << labels_[j];
		}

		for (unsigned int i = 0; i < quaternionData_.size(); ++i) { // iteration through rows
			// after the first 2 rows of text, start with a new line and put time values in the first column
			outputFile << "\n" << std::setprecision(outputPrecision_) << timeVector_[i];
			for (unsigned int j = 0; j < labelsSize_; ++j) {
				// then input quaternion values, separating them from time and other quaternion values with a tab
				outputFile << "\t" << quaternionData_[i][j];
			}
		}
		outputFile.close();
		std::cout << "Quaternion time series written to file " << filePath << std::endl;
	}
	else {
		std::cout << "Failed to open file " << filePath << std::endl;
	}
}



void MpuDataReader::updateTime() {
	// if we are going to save quaternions to file later, push current quaternions to relevant vectors and update timevector
	if (saveQuaternionsToFile_) {
		// push quaternions into vector
		quaternionData_.push_back(quaternions_);
		// get current time
		clockNow_ = std::chrono::high_resolution_clock::now();
		// update elapsed time (clockDuration_)
		clockDuration_ = (clockNow_ - clockStart_);
		// push time into vector
		timeVector_.push_back(clockDuration_.count());
	}
}