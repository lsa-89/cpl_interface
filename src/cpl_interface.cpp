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

// data structure to keep track of all incoming queries, we use unordered_map for cheaper inserts/accesses
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
    if (queries.count(req.id) > 0 || processed_queries.count(req.id) > 0) {
        res.ok = false;
        res.message = "Another query is already being processed with id " + req.id;
        return true;
    } else {
        int mode = req.mode;
        std::string id = req.id;
        std::string query_string = req.query;

        plIfaceGlobal->PrologInterface::push_query(mode, id, query_string, false);

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

    /*
     * no id entered?
     */
    if (req.id == "") {
        res.status = 1; // 1 = wrong id
        res.solution = "";
        return true;
    }

    /*
     * no query for id?
     */
    std::unordered_map<std::string, PrologQuery>::iterator iterator;
    iterator = processed_queries.find(req.id);

    if (iterator == processed_queries.end()) {
        res.status = 1; // 1 = wrong id
        res.solution = ""; // should the solution be empty if the wrong ID is used?
        return true;
    }

    /*
     * id seems to be fine, get the next solution
     */
    PrologQuery query = plIfaceGlobal->PrologInterface::pop_query(req.id);

    plIfaceGlobal->PrologInterface::push_query(query.get_mode(), query.get_id(), query.get_query(), true);

    std::string thread_id = query.get_pl_thread_id();

    std::cout << "The ID of the pl thread is: " + thread_id << std::endl;

//    std::string next_solution;
//    bool has_next_solution = has_next_solution();
//
//    if (has_next_solution) {
//        next_solution = get_next_solution(thread_id);
//    }

    /*
     * there is no next solution anymore
     */
//    else {
//        res.status = 0; // 0 = no solution
//        res.solution = "";
//        return true;
//    }

    // hole alte query raus aus proc.queries

    //push geupdatete query wieder zu queries -> wakeup vom worker thread



//        next speichern
//        next returnen


    res.status = 3; // 3 = ok
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
    //PLTerm foo;
    //av[1] = foo;
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

void PrologInterface::push_query(int mode, std::string id, std::string query, bool request_next_solution) {

    // build a query from the requirements
    PrologQuery queryObj;
//    std::string query_string = query;
    // set_values(mode, id, query, msg, ok)
    queryObj.set_mode(mode);
    queryObj.set_id(id);
    queryObj.set_pl_thread_id("");
    queryObj.set_query(query);
    queryObj.set_message("");
    queryObj.set_solution("");
    queryObj.set_ok(true);
    queryObj.set_request_next_solution(request_next_solution);

    push_lock.lock();
    queries.insert({id, queryObj}); // synchronized push of the query to the shared map of queries
    push_lock.unlock();

    has_queries_to_process = true;
    cv_loop.notify_one();
    ROS_INFO("PUSH QUERY PASSED");
}

PrologQuery PrologInterface::pop_query(std::string id) {

    std::unordered_map<std::string, PrologQuery>::iterator iterator;
    PrologQuery popped_query;

    /*
     * check processed_queries for a matching instance
     */
    iterator = processed_queries.find(id);
    if (!(iterator == processed_queries.end())) {
        push_lock.lock();
        popped_query = processed_queries[id];
        processed_queries.erase(iterator);
        push_lock.unlock();
    }
        /*
         * maybe the matching query hasn't been transferred to processing_queries yet,
         * so we check the unprocessed queries as well
         */
    else {
        iterator = queries.find(id);
        if (!(iterator == queries.end())) {
            push_lock.lock();
            popped_query = queries[id];
            queries.erase(iterator);
            push_lock.unlock();
        }
    }
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
}

std::string pl_next_solution(std::shared_ptr <PlEngine> engine, std::string thread_id) {


    // Check if more solutions are available
    std::string next_solution;

    std::cout << "pl_next_solution reached" << std::endl;
    PlFrame fr;
    PlTermv av(2);
    av[0] = thread_id.c_str(); //PlCompound(thread_id.c_str());

//    PlTerm av(PlCompound(thread_id.c_str()));

    try {
        std::cout << "PL_Thread_ID for next solution: " << (char *) av[0] << std::endl;
        PlQuery q("queryt_next_solution", av);
        while (q.next_solution()) {
            //PlTail solution_list(av[1]);
            //json_parse_list(solution_list, json_object);
            //print_json_object(json_object);
            // {
            //   'A': JSON_LIST([1,2,3]
            //   'B': ....
            // }

//        while () {
//            std::cout << "Got into the while loop -- so there is at least one solution." << std::endl;

            std::cout << "foo1: " << std::endl;  // 'A'
            std::cout << "foo1: " << (char *) av[1] << std::endl;  // 'A'
            PlTail solution_list(av[1]);
            std::cout << "foo2: " << std::endl;  // 'A'
            PlTerm assignment_term;
            solution_list.next(assignment_term);
            std::cout << "foo3: " << (char *) assignment_term << std::endl;  // 'A'
            solution_list.close();

            PlTail assignment(assignment_term);
            PlTerm name_term, value_term;
            assignment.next(name_term);
            std::cout << "term0: " << (char *) name_term << std::endl;  // 'A'
            assignment.next(value_term);
            std::cout << "term1: " << (char *) value_term << std::endl; // [1,2,3]

            PlTail test_value_list(value_term);
            PlTerm list_value_term;
            test_value_list.next(list_value_term);
            std::cout << "term1.1: " << (char *) test_value_list << std::endl;  // 'A'
            std::cout << "term1.2: " << (char *) list_value_term << std::endl;  // 'A'

            assignment.close();

            //std::cout << (char *) av[1] << std::endl;
            // next_solution.assign(av[1]);
        }
        std::cout << "DONE: " << std::endl;  // 'A'

//        }
    }
    catch (PlException &ex) {
        ROS_INFO((char *) ex);
    }

    ROS_INFO(next_solution.c_str());
    return next_solution;
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

                iterator = queries.begin();
                std::string query_string(iterator->second.get_query());

//                std::cout
//                if (iterator->second.get_request_next_solution()) {
                std::string rosinfo = "Received query: " + query_string;
                ROS_INFO(rosinfo.c_str());
                std::string thread_id = pl_threaded_call(engine, query_string);
                push_lock.lock();
                iterator->second.set_pl_thread_id(thread_id);
                push_lock.unlock();
                ROS_INFO(thread_id.c_str());
                pl_next_solution(engine, thread_id);
//                }
//                else {
//                    std::string thread_id = iterator->second.get_pl_thread_id();
//                    pl_next_solution(engine, thread_id);
//                }


//                ROS_INFO(thread_id.c_str());
//                if (has_next_solution(engine, thread_id)) {
//                    std::cout << "true" << std::endl;
//                }


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