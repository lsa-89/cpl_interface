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
std::unordered_map <std::string, PrologQuery> processed_queries;

//std::condition_variable cv_loop;

// shared mutex, used by query threads
std::mutex push_lock;

// global access to the prolog interface instance
PrologInterface *plIfaceGlobal = NULL;

bool query(json_prolog_msgs::PrologQuery::Request &req,
           json_prolog_msgs::PrologQuery::Response &res) {

    // case id already exists in queries
    if (queries.count(req.id) > 0) {
        res.ok = false;
        res.message = "Another query is already being processed with id" + req.id;
    } else {


        int mode = req.mode;
        std::string id = req.id;
        std::string query_string = req.query;

        plIfaceGlobal->PrologInterface::push_query(mode, id, query_string);

        res.ok = true; //query.get_ok();
        res.message = ""; //query.get_message();

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

    if (req.id == "") {
        res.status = 1;
        res.solution = "";
        return true;
    }

    PrologQuery query = plIfaceGlobal->PrologInterface::pop_query(req.id);

//    if (query == NULL) {
//        res.byte = 2;
//        res.solution = "";
//        return true;
//    }

    res.status = 3;
    res.solution = "";
    return true;
}

/*
 * Make a call to prolog that is started in a new prolog thread, using
 * the threaded_query.pl interface
 */
std::string pl_threaded_call(std::shared_ptr <PlEngine> engine, std::string input) {

    /*
     * Builds up the query for prolog. The following example should form
     * a prolog query of the form
     *
     * call(thread_create, write(1), Id, []).
     */

    std::string query_string = input;// + ", Id";
    std::string thread_id;

    /*
     * This is the prolog term that the prolog engine will try to unify.
     * av[1] will yield the solution to the imposed query.
     *
     * The query that's being send to prolog, in this case, will look like this:
     * queryt_create('<input>', Id).
     */
    PlFrame fr;
    PlTermv av(2);
    av[0] = input.c_str(); //PlString(query_string.c_str());
//    av[1] = "Id";
//    av[0] = PlCompound(query_string.c_str());

    try {
        PlQuery q("queryt_create", av);

        while (q.next_solution())
            thread_id.assign(av[1]);
    }
    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }
    return thread_id;
}

void PrologInterface::push_query(int mode, std::string id, std::string query) {

    // build a query from the requirements
    PrologQuery queryObj;
    std::string query_string = query;
    // set_values(mode, id, query, msg, ok)
    queryObj.set_values(mode, id, query_string, "", true);

    push_lock.lock();
    queries.insert({id, queryObj}); // synchronized push of the query to the shared map of queries
    push_lock.unlock();

    has_queries_to_process = true;
    cv_loop.notify_one();
    ROS_INFO("PUSH QUERY PASSED");
}

PrologQuery PrologInterface::pop_query(std::string id) {

    PrologQuery popped_query = processed_queries[id];

    return popped_query;
}

void PrologInterface::init() {

    PlTerm av("knowrob_common");
    try {
        PlCall("register_ros_package", av);
    }
    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }

//    PlTermv out(4); // Diese Zeile wirft seg fault wird aber im example auch so verwendet
//    out[0] = "thread_create";
//    out[1] = "write(1)";
//    out[2] = "Id";
//    out[3] = "[]";
//
//    try {
//        PlQuery q("call", out);
//    }
//    catch (PlException &ex) {
//        ROS_INFO((char *) ex);
//    }
}

void has_next_solution(std::shared_ptr <PlEngine> engine, std::string thread_id) {

    std::string solution;
//    PlFrame fr;
    PlTermv av(2);
    av[0] = thread_id.c_str(); //PlCompound(thread_id.c_str());

//    PlTerm av(PlCompound(thread_id.c_str()));

    try {
        std::cout << "Got into the try block." << std::endl;
        PlQuery q("queryt_has_next", av);

//        while () {
//            std::cout << "Got into the while loop -- so there is at least one solution." << std::endl;
            std::cout << (char *) av[1] << std::endl;
            solution.assign(av[1]);
//        }
    }
    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }
    ROS_INFO(solution.c_str());

}

void PrologInterface::loop() {

    ROS_INFO("Invoking prolog engine");
    char *argv[4];
    int argc = 0;
    argv[argc++] = (char *) "PrologEngine"; // cast only to solve warnings from the compiler
    argv[argc++] = (char *) "-f";
    std::string rosPrologInit = ros::package::getPath("rosprolog") + "/prolog/init.pl";
    argv[argc] = new char[rosPrologInit.size() + 1];
    std::copy(rosPrologInit.begin(), rosPrologInit.end(), argv[argc]);
    argv[argc++][rosPrologInit.size()] = '\0';
    argv[argc] = NULL;

    engine = std::make_shared<PlEngine>(argc, argv);
    init();

    std::unordered_map<std::string, PrologQuery>::iterator iterator;
    std::string debug_msg = "";
    while (ros::ok()) {
        {
            std::unique_lock <std::mutex> lk(loop_lock);
            // queries available?
            while (!queries.empty()) {
//                debug_msg = "ENTER WHILE LOOP: " + std::to_string(counter);

                iterator = queries.begin();
                std::string query_string(iterator->second.get_query());

                ROS_INFO(query_string.c_str());
                // take first query from list, get value (the query) and get the query string
                std::string thread_id = pl_threaded_call(engine, query_string);

                ROS_INFO(thread_id.c_str());
//                if (has_next_solution(engine, thread_id)) {
//                    std::cout << "true" << std::endl;
//                }

                has_next_solution(engine, thread_id);
//                thread_id += + ", Next";
//                std::string solution;
//                PlFrame fr;
//                PlTermv av(2);
//                av[0] = PlCompound(thread_id.c_str());
//
//
//
//                try {
//                    std::cout << "Got into the try block." << std::endl;
//                    PlCall ("queryt_next_solution", av);
////                    PlQuery q("queryt_next_solution", av);
//
//                    while (plt_has_next(thread_id)) {
//                        std::cout << "Got into the while loop -- so there is at least one solution." << std::endl;
//                        std::cout << (char *) av[1] << std::endl;
//                        solution.assign(av[1]);
//                    }
//                }
//                catch (PlException &ex) {
//                    ROS_INFO((char *) ex);
//                }
//
//                ROS_INFO(solution.c_str());

                push_lock.lock();
                processed_queries.insert({iterator->first, iterator->second});
                queries.erase(iterator); // remove query from the queued queries
                push_lock.unlock();

            }
            ROS_INFO("WAIT FOR QUERIES...");
            cv_loop.wait(lk, [this] { return has_queries_to_process; });
            ROS_INFO("WAKE UP - QUERIES AVAILABLE.");
            has_queries_to_process = false;
        }
    }
}

PrologInterface::PrologInterface() :
        thread(&PrologInterface::loop, this) {
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
    plIfaceGlobal = &prologInterface;

    ros::ServiceServer service_query = n.advertiseService("query", query);
    ros::ServiceServer service_next_solution = n.advertiseService("next_solution", next_solution);
    ros::ServiceServer service_finish = n.advertiseService("finish", finish);
    ros::spin();

    return 0;
}