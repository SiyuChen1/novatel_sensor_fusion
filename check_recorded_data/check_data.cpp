// g++ -std=c++11 -O3 -o check_data check_data.cpp

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

enum DataType{
    NONDEFINED=-1,
    CORRIMUS=0,
    INSATTQS=1,
    BESTGNSSVEL=2,
    BESTGNSSPOS=3,
    BESTVEL=4,
    BESTPOS=5,
    BESTXYZ=6,
    BESTUTM=7,
    GPRMC=8,
    GPGGA=9,
    RAWIMU=10
};

const double THRESHOLD = 1e-8;

void get_header_data(std::string& line,
                     std::vector<std::string>& header,
                     std::vector<std::string>& data){
    int last = 0;
    bool is_data = false;
    for (int i = 0; i <= line.size(); i++) {
        // If we reached the end of the word or the end of the input.
        if (line[i] == ',' || line[i] == ';' || i == line.size()){
            std::string tmp = line.substr(last, i-last);
            if(!is_data){
                header.push_back(tmp);
            } else{
                data.push_back(tmp);
            }
            last = i + 1;
            if (line[i]== ';')
                is_data = true;
        }
    }

    // for GPRMC and GPGGA log, there is no delimiter ';'
    // just treat header as data
    if(data.size() == 0){
        data = header;
    }
}

void get_log_type(const std::vector<std::string>& header,
                  DataType& type){
    int size = header[0].size();
    std::string type_;
    for(int i = 0; i< size; i++){
        char tmp = header[0][i];
        if(tmp == '%' || tmp == '#' || tmp == '$'){
            type_ = header[0].substr(i+1, size - i - 1);
            break;
        }
    }
    // remove last character 'A' ascii or 'B' binary, except GPGGA
    if(type_!= "GPGGA" && (type_.back() == 'A' || type_.back() == 'B')){
        type_.pop_back();
    }

    if(type_ == "CORRIMUS"){
        type = DataType::CORRIMUS;
    }else if (type_ == "INSATTQS"){
        type = DataType::INSATTQS;
    }else if (type_ == "BESTGNSSVEL"){
        type = DataType::BESTGNSSVEL;
    }else if (type_ == "BESTGNSSPOS"){
        type = DataType::BESTGNSSPOS;
    }else if (type_ == "BESTVEL"){
        type = DataType::BESTVEL;
    }else if (type_ == "BESTPOS"){
        type = DataType::BESTPOS;
    }else if (type_ == "BESTXYZ"){
        type = DataType::BESTXYZ;
    }else if (type_ == "BESTUTM"){
        type = DataType::BESTUTM;
    }else if (type_ == "GPRMC"){
        type = DataType::GPRMC;
    }else if (type_ == "GPGGA"){
        type = DataType::GPGGA;
    }else if (type_ == "RAWIMU" || type_ == "RAWIMUS"){
        type = DataType::RAWIMU;
    }else{
        type = DataType::NONDEFINED;
    }
}


// if acceleration and angular velocity are close to 1e-8
// this log is invalid, otherwise valid
bool check_corrimus(const std::vector<std::string>& data){
    int flag = 0;
    for(int id = 1; id < 7; id++) {
        std::stringstream ss(data[id]);
        double val;
        if (ss >> val) {  // Successfully converted to double
            if (std::abs(val) < THRESHOLD){
                flag ++;
            }
        }
    }
    return flag < 6 ? true: false;
}

// https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
// page 1113
bool check_insattqs(const std::vector<std::string>& data){
    std::string status = data[6];
    int i = 0;
    for(; i < status.size(); i ++){
        if(status[i] == '*')
            break;
    }
    status = status.substr(0, i);
    return status == "INS_SOLUTION_GOOD" ? true : false;
}

// https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
// page 1094, 542, 543
bool check_bestgnssvel(const std::vector<std::string>& data){
    return data[0] == "SOL_COMPUTED" && data[1] == "NARROW_INT" ? true : false;
}

// https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
// page 1092, 542, 543
bool check_bestgnsspos(const std::vector<std::string>& data){
    return data[0] == "SOL_COMPUTED" && data[1] == "NARROW_INT" ? true : false;
}

bool check_bestvel(const std::vector<std::string>& data){
    return data[0] == "SOL_COMPUTED" && data[1] == "INS_RTKFIXED" ? true : false;
}

bool check_bestpos(const std::vector<std::string>& data){
    return data[0] == "SOL_COMPUTED" && data[1] == "INS_RTKFIXED" ? true : false;
}

bool check_bestxyz(const std::vector<std::string>& data){
    return data[0] == "SOL_COMPUTED" && data[1] == "INS_RTKFIXED" ? true : false;
}

bool check_bestutm(const std::vector<std::string>& data){
    return data[0] == "SOL_COMPUTED" && data[1] == "INS_RTKFIXED" ? true : false;
}

// https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
// page 653, Position status (A = data valid, V = data invalid)
bool check_gprmc(const std::vector<std::string>& data){
    return data[2] == "A" ? true : false;
}

// https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
// page 631, table 119, 4 means RTK fixed ambiguity solution
bool check_gpgga(const std::vector<std::string>& data) {
    return data[6] == "4" ? true : false;
}

// https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
// page 1183, table 268, it seems that recorded IMU status is always invalid
bool check_rawimu(const std::vector<std::string>& data){
    return true;
}

int main(int argc, char* argv[]) {
    for (int i = 0; i < argc; ++i) {
        std::cout << argv[i] << "\n";
    }

    std::vector<bool(*)(const std::vector<std::string>&)> functions(11);
    functions[DataType::CORRIMUS] = &check_corrimus;
    functions[DataType::INSATTQS] = &check_insattqs;
    functions[DataType::BESTGNSSVEL] = &check_bestgnssvel;
    functions[DataType::BESTGNSSPOS] = &check_bestgnsspos;
    functions[DataType::BESTVEL] = &check_bestvel;
    functions[DataType::BESTPOS] = &check_bestpos;
    functions[DataType::BESTXYZ] = &check_bestxyz;
    functions[DataType::BESTUTM] = &check_bestutm;
    functions[DataType::GPGGA] = &check_gpgga;
    functions[DataType::GPRMC] = &check_gprmc;
    functions[DataType::RAWIMU] = &check_rawimu;

    std::ifstream file(argv[1]);
    std::string line;

    std::vector<int> valid(11, 0), total(11, 0);

    if (file.is_open()) {
        while (std::getline(file, line)) {
            // Process each line as needed
            if(!line.empty()){
                std::vector<std::string> data;
                std::vector<std::string> header;
                get_header_data(line, header, data);
                DataType type;
                get_log_type(header, type);
                if (type > -1){

                    auto fun = functions[type];
                    bool flag = fun(data);
                    total[type] ++;
                    valid[type] = valid[type] + (int) flag;
                }
            }
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }

    std::vector<std::string> datatype_names = {
        "CORRIMU",
        "INSATTQ",
        "BESTGNSSVEL",
        "BESTGNSSPOS",
        "BESTVEL",
        "BESTPOS",
        "BESTXYZ",
        "BESTUTM",
        "GPRMC",
        "GPGGA",
        "RAWIMU(S)"
    };

    std::vector<std::string> datatype_info = {
        "Not all IMU data close to 0",
        "INS_SOLUTION_GOOD",
        "SOL_COMPUTED & NARROW_INT",
        "SOL_COMPUTED & NARROW_INT",
        "SOL_COMPUTED & INS_RTKFIXED",
        "SOL_COMPUTED & INS_RTKFIXED",
        "SOL_COMPUTED & INS_RTKFIXED",
        "SOL_COMPUTED & INS_RTKFIXED",
        "POS STATUS == A means valid",
        "GPS Quality == 4 means RTK_FIXED",
        "RAWIMU NO CHECKING"
    };

    // Open the file in write mode
    std::ofstream out_file("summary.txt");

    // Backup the standard output stream buffer
    std::streambuf* coutbuf = std::cout.rdbuf();
    // Redirect cout to file
    std::cout.rdbuf(out_file.rdbuf());

    std::cout << std::left << std::setw(12) << "Log" << std::setw(12)
        << "Total" << std::setw(12) << "Valid" << std::setw(12) << "Criteria" << std::endl;
    for (size_t i = 0; i < total.size(); ++i) {
        std::cout << std::left << std::setw(12) << datatype_names[i] << std::setw(12)
        << total[i] << std::setw(12) << valid[i] << std::setw(12) << datatype_info[i] << std::endl;
    }

    // Restore the original buffer so that output goes to the terminal again
    std::cout.rdbuf(coutbuf);

    // Close the file
    out_file.close();

    return 0;

}
