#include "ros/ros.h"
#include "json_prolog_msgs/PrologQuery.h"
#include "json_prolog_msgs/PrologFinish.h"
#include "json_prolog_msgs/PrologNextSolution.h"
#include "cpl_interface.h"
#include "ros/package.h"
#include "time.h"
#include <thread>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <condition_variable>

// data structure to keep track of all incoming queries, we use unorderd_map for cheaper inserts/accesses
// map: considers ordering, O(log n) for insert/access
// unordered_map: no ordering, O(1) for insert/access
std::unordered_map <std::string, PrologQuery> queries; // shared set of queries
std::mutex hartmut; // mutex for synchronization of accesses to queries
std::condition_variable cv_loop;
std::condition_variable cv;
std::condition_variable cv_work;

bool query(json_prolog_msgs::PrologQuery::Request &req,
           json_prolog_msgs::PrologQuery::Response &res) {

    // case id already exists in queries
    if (queries.count(req.id) > 0) {
        res.ok = false;
        res.message = "Another query is already being processed with id" + req.id;
    } else {
        // build a query from the requirements
        PrologQuery query;
        query.set_values(req.mode, req.id, req.query, NULL, NULL);

        typedef std::unordered_map<std::string, PrologQuery>::iterator UOMIterator;
        std::pair<UOMIterator, bool> insertionPair;


        std::unique_lock<std::mutex> locker(hartmut);
        hartmut.lock();
        insertionPair = queries.insert({req.id, query}); // synchronized push of the query to the shared map of queries
        hartmut.unlock();
        cv_work.notify_all();
        ROS_INFO(insertionPair.first->second.get_query().c_str());
        cv.wait(locker); // let this thread sleep until it is notified that the query has been processed

        // set the query from the insertion iterator, so we don't need to do a lookup in the map
        query = insertionPair.first->second;

        res.ok = query.get_ok();
        res.message = query.get_message();

        return true;
    }
}

bool finish(json_prolog_msgs::PrologFinish::Request &req,
            json_prolog_msgs::PrologFinish::Response &res) {
    //res.sum = req.a + req.b;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

bool next_solution(json_prolog_msgs::PrologNextSolution::Request &req,
                   json_prolog_msgs::PrologNextSolution::Response &res) {
    //res.sum = req.a + req.b;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

void pl_threaded_call(const std::string input) {

//    std::string query_string = input + ", Id, []";

    PlTerm argv(input.c_str());
    try {
        PlQuery q("cpl_proof_of_concept", argv);
    }
    catch (PlException &ex) {
        ROS_INFO("Prolog test call went wrong. You're not there yet, pal.");
    }

//    PREDICATE(list_modules, 0)
//    { PlTermv av(1);
//
//        PlQuery q("current_module", av);
//        while( q.next_solution() )
//            cout << (char *)av[0] << endl;
//
//        return TRUE;
//    }
}

//void testWithoutThreading() {
//    for (int i = 0; i < 1000; ++i)
//        makeDummyCall("hello");
//}

void testWithThreading() {

    std::list <std::thread> threads;

//    for (int i = 0; i < 1000; ++i) {
//        //std::thread t(makeDummyCall, "doesntmatter");
//        threads.push_back(std::thread( makeDummyCall, "something") );
//        //t.detach();
//    }
    std::thread first(pl_threaded_call, "String");
    first.join();
//    for (std::thread & t : threads) {
//        t.join();
//    }

}

void PrologInterface::init() {
    //ROS_INFO("Initializing Prolog Engine");

    PlTerm av("knowrob_common");
    try {
        PlCall("register_ros_package", av);
    }
    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }
}

void PrologInterface::loop() {
    while(1){
        {
            std::unique_lock<std::mutex> lk(loop_lock);
            // queries available?
            while(!queries.empty()) {
                ROS_INFO("I GOT HERE *****");
                std::string query_string(queries.begin()->second.get_query());

                // take first query from list, get value (the query) and get the query string
                pl_threaded_call(query_string);
            }
            cv_loop.wait(lk, [this]{ return has_queries_to_process; });
            has_queries_to_process = false;


            }

        }
    }

PrologInterface::PrologInterface() :
    thread(&PrologInterface::loop, this){

    ROS_INFO("Invoking prolog engine");
    char *argv[4];
    int argc = 0;
    argv[argc++] = "PrologEngine";
    argv[argc++] = "-f";
    std::string rosPrologInit = ros::package::getPath("rosprolog") + "/prolog/init.pl";
    argv[argc] = new char[rosPrologInit.size() + 1];
    std::copy(rosPrologInit.begin(), rosPrologInit.end(), argv[argc]);
    argv[argc++][rosPrologInit.size()] = '\0';
    argv[argc] = NULL;
    engine = std::make_shared<PlEngine>(argc, argv);
    init();
}

void doSequence() {
    struct timespec start, finish;
    double elapsed;

    clock_gettime(CLOCK_REALTIME, &start);
    clock_gettime(CLOCK_REALTIME, &finish);

    elapsed = (finish.tv_sec - start.tv_sec);
    elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

    std::cout << "Time elapsed: " + std::to_string(elapsed) << std::endl;
}

void doMultiThreading() {

    struct timespec start, finish;
    double elapsed;
    std::list <std::thread> threads;

    clock_gettime(CLOCK_REALTIME, &start);
    for (int i = 0; i < 2; ++i) {
        //threads.push_back(std::thread(  ) );
    }
    for (std::thread &t : threads) {
        t.join();
    }
    clock_gettime(CLOCK_REALTIME, &finish);

    elapsed = (finish.tv_sec - start.tv_sec);
    elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

    std::cout << "Time elapsed: " + std::to_string(elapsed) << std::endl;
}

void fun1(int input) {
    int j = 0;

    input += j;
    std::cout << input << std::endl;
}

void fun2(int input) {
    int j = 5;

    input -= j;
    std::cout << input << std::endl;
}

void threadingTest() {

    std::list <std::thread> threads;

    for (int i = 0; i < 200; ++i)
        threads.push_back(std::thread(fun1, 234));

    for (std::thread &t : threads)
        t.join();
}

void PrologQuery::set_values(int p_mode, const std::string &p_id,
                             const std::string &p_query, const std::string &p_message, bool p_ok) {
    mode = p_mode;
    id = p_id;
    query = p_query;
    message = p_message;
    ok = p_ok;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cpl_interface_node");
    ros::NodeHandle n;

    // create the thread handler, that checks for queries and passes them to prolog
    PrologInterface prologInterface;
//    std::thread handler(PrologInterface);

    ros::ServiceServer service_query = n.advertiseService("query", query);
    ros::ServiceServer service_next_solution = n.advertiseService("next_solution", next_solution);
    ros::ServiceServer service_finish = n.advertiseService("finish", finish);
    ros::spin();

    return 0;
}