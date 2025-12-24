import argparse
from importlib import import_module
from multiprocessing import Process
import os.path
import sqlite3
import sys
from threading import Thread

from lazy_pyros_utils.enums import Boolean, PullEmpty, PushFull, Wrap
from lazy_pyros_utils.ioproc import generate_dir_name, get_input, get_input_from_sql, print_pretty_sql, sanitise_yes_no, throw_tantrum
from lazy_pyros_utils.topic import create_queue, create_queue_mp
from lazy_pyros_utils.wrapper import class_wrapper, func_wrapper, workdir_wrapper

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_db", help="Path to SQLite database file to be opened")
    parser.add_argument("--manual", help="Manually input type and node IDs to skip user prompts, in the format <typeID1>-<nodeID1>_<typeID2>-<nodeID2>_...")
    args = parser.parse_args()
    db_path = args.path_to_db

    if not os.path.exists(db_path):
         raise Exception(str(db_path)+" does not exist. Create a database using db_create.py first.")

    conn = sqlite3.connect(db_path)

    # for testing
    # conn = sqlite3.connect("/mnt/c/Users/UserAdmin/Downloads/main.db")

    conn.execute("PRAGMA foreign_keys = 1")
    cursor = conn.cursor()

    import_types = []
    import_nodes = []
    if args.manual:
        type_node_str_list = args.manual.split('_')
        for type_node in type_node_str_list:
            type_id, node_id = type_node.split('-')
            type_id, node_id = int(type_id), int(node_id)
            import_types.append(type_id)
            import_nodes.append(node_id)
    else:
        type_id = get_input_from_sql("Add a Node Type ID:", cursor, "SELECT * FROM type", ["ID", "Name", "Description"], title="List of Types")
        while type_id != 0:
            skip_to_end = False
            if type_id in import_types:
                throw_tantrum("Only one instance of each node type is allowed.")
                skip_to_end = True
            
            if not skip_to_end:
                try:
                    node_id = get_input_from_sql("Node Type "+str(type_id)+" - Choose a Node ID:", cursor, "SELECT id, name, desc FROM node WHERE type_id=?", ["ID", "Name", "Description"], (type_id,), title="List of Nodes for Type "+str(type_id))
                except Exception as err:
                    print("Select a different node type, or add a node for this node type.")
                    throw_tantrum(str(err))
                    skip_to_end = True
            
            if not skip_to_end:
                import_types.append(type_id)
                import_nodes.append(node_id)

            type_id = get_input_from_sql("Add a Node Type ID (enter 0 to continue):", cursor, "SELECT * FROM type", ["ID", "Name", "Description"], extra_ans=[0,], title="List of Types")
        del type_id

        print_pretty_sql(cursor, f"SELECT type.id, type.name, type.desc, node.id, node.name, node.desc, node.func FROM node JOIN type ON node.type_id=type.id WHERE node.id IN ({','.join(['?']*len(import_nodes))})", ["Type ID", "Type Name", "Type Description", "Node ID", "Node Name", "Node Description", "Target Function"], import_nodes, "Your Selected Nodes", 20)
        if get_input("\nConfirm selected nodes? [y/n]", sanitise=sanitise_yes_no) == 'n':
            print("Task abandoned!")
            conn.close()
            sys.exit()
    
    # asserted on input that each type only has one node
    topics_in_list = [[] for i in import_types] # 1 list of queues per type/node
    topics_out_list = [[] for i in import_types] # 1 list of queues per published topic per type/node
    subscriptions = {}
    mp_topics = []

    # set up all published topics
    for i in range(len(import_types)):
        type_id = import_types[i]
        node_id = import_nodes[i]
        cursor.execute("SELECT topic_id FROM type_publish_topic WHERE type_id=? ORDER BY id", (type_id,))
        topics_out = cursor.fetchall()
        cursor.execute("SELECT multiproc FROM node WHERE id=?", (node_id,))
        node_mp = cursor.fetchall()[0][0]
        for topic in topics_out:
            topic_id = topic[0]
            if topic_id in subscriptions:
                conn.close()
                raise Exception("More than one node is publishing the same topic.")
            topics_out_list[i].append(topic_id)
            subscriptions[topic_id] = []
            if node_mp == Boolean.TRUE_: mp_topics.append(topic_id)

    target_funcs = []
    thread_params = []
    for i in range(len(import_types)):
        node_id = import_nodes[i]
        cursor.execute("SELECT node.type_id, type.name, node.id, node.name, node.pkg_name, node.func, node.wrap, node.method, node.daemon, node.multiproc, node.workdir FROM node JOIN type ON node.type_id=type.id WHERE node.id=?", (node_id,))
        node_info = cursor.fetchall()

        type_id = node_info[-1][0]
        type_name = node_info[-1][1]
        node_id = node_info[-1][2]
        node_name = node_info[-1][3]
        pkg_name = node_info[-1][4]
        func_name = node_info[-1][5]
        wrap = Wrap(node_info[-1][6])
        method = node_info[-1][7]
        daemon = Boolean(node_info[-1][8])
        multiproc = Boolean(node_info[-1][9])
        workdir = node_info[-1][10]

        thread_params.append((daemon, multiproc))

        # import target functions
        type_dir = generate_dir_name("type", type_id, type_name)
        node_dir = generate_dir_name("node", node_id, node_name)
        import_mod_name = type_dir + "." + node_dir
        if workdir: # compatibility if using absolute imports
            workdir = os.path.join(os.path.dirname(__file__), type_dir, node_dir, workdir)
            sys.path.insert(0, workdir)
        # when file name is same as function name, you cannot use import file (or apparently can?)
        if pkg_name:
            import_mod_name += "." + pkg_name
            target = getattr(import_module(import_mod_name), func_name)
        else:
            target = getattr(import_module(import_mod_name), func_name)

        # wrap target functions
        if wrap == Wrap.FUNC:
            target_funcs.append(func_wrapper(target, workdir))
        elif wrap == Wrap.CLASS:
            target_funcs.append(class_wrapper(target, method, workdir))
        elif workdir:
            target_funcs.append(workdir_wrapper(target, workdir))
        else:
            target_funcs.append(target)

        # create queues and raise errors for missing types
        cursor.execute("SELECT topic_id, size, pushfull, pullempty FROM node_subscribe_topic WHERE node_id=?", (node_id,))
        topic_table = cursor.fetchall()
        for topic in topic_table:
            topic_id = topic[0]
            max_size = topic[1]
            pushfull = PushFull(topic[2])
            pullempty = PullEmpty(topic[3])

            if topic_id not in subscriptions:
                conn.close()
                raise Exception("Topic "+str(topic_id)+" not found.")
            
            if multiproc == Boolean.TRUE_ or topic in mp_topics:
                new_topic = create_queue_mp(pushfull, pullempty, max_size)
            else:
                new_topic = create_queue(pushfull, pullempty, max_size)

            topics_in_list[i].append(new_topic)
            subscriptions[topic_id].append(new_topic)
    
    conn.close()
    topics_out_list = [[subscriptions[sub] for sub in out_list] for out_list in topics_out_list]
    
    any_daemon = False
    for i in range(len(import_types)):
        daemon = True if thread_params[i][0] == Boolean.TRUE_ else False
        multiproc = thread_params[i][1]

        # if all threads are daemon threads, they will stop at the end of run.py
        # last thread is non-daemonic if all other threads are daemonic
        if daemon == False: any_daemon = True
        if any_daemon == False and i == len(import_types) - 1: dameon = False

        if multiproc:
            Process(target=target_funcs[i], args=(topics_in_list[i], topics_out_list[i]), daemon=daemon).start()
        else:
            Thread(target=target_funcs[i], args=(topics_in_list[i], topics_out_list[i]), daemon=daemon).start()

    print("Everything is running!")