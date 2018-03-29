#ifndef JSONPROLOGINTERFACE_H
#define JSONPROLOGINTERFACE_H

//SWI Prolog
#include <SWI-cpp.h>
//ros
#include <ros/package.h>
//STD
#include <memory>
#include <iostream>
#include <unordered_map>

//wrapper class for Prolog Engine based on SWI-C++
class PrologInterface {
    typedef std::shared_ptr <PlEngine> PlEnginePtr;
    PlEnginePtr engine;
    //PlEngine* engine;
    bool useJsonProlog;
    int useThreading;

public:
    PrologInterface(bool json_prolog = false);

    ~PrologInterface() {
    }

    /*brief
     * initialize the necessary knowrob packages
     */
    void init();

};

class PrologQuery {
    int mode;
    std::string id, query, message;
    bool ok;


public:
    void set_values(int p_mode, const std::string &p_id,
                    const std::string &p_query,
                    const std::string &p_message, bool p_ok);

    const std::string &get_message() { return message; }

    bool get_ok() { return ok; }

    ~PrologQuery() {
    }
};

PrologInterface prologInterface;

#endif //JSONPROLOGINTERFACE_H
