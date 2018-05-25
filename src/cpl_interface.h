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
#include <thread>
#include <mutex>
#include <condition_variable>

//wrapper class for Prolog Engine based on SWI-C++

class PrologQuery {
    int mode;
    std::string id, query, message;
    bool ok;


public:
    void set_values(int p_mode, const std::string &p_id,
                    const std::string &p_query,
                    const std::string &p_message, bool p_ok);

    std::string &get_query() { return query;}

    std::string &get_id() {return id;}

    std::string &get_message() { return message; }

    bool get_ok() { return ok; }

    ~PrologQuery() {
    }
};

class PrologInterface {
private:
    typedef std::shared_ptr <PlEngine> PlEnginePtr;
    PlEnginePtr engine;
    std::mutex loop_lock;
    std::condition_variable cv_loop;
    std::thread thread;
    bool has_queries_to_process = false;


public:
    PrologInterface();

    ~PrologInterface() {
    }

    /*brief
     * initialize the necessary knowrob packages
     */
    void init();

    /*
     * main loop to process queries
     */
    void loop();

    /*
     * push a query to the map of queries
     */
    void push_query(int mode, std::string id, std::string query);

    /*
     * pop a query from the map of queries
     */
    PrologQuery pop_query(std::string);
};

#endif //JSONPROLOGINTERFACE_H
