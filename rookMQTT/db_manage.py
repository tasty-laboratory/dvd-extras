import argparse
import os.path
import sqlite3

from lazy_pyros_utils.enums import Boolean, PullEmpty, PushFull, PushFullDisplayBlock, PushFullDisplayNoBlock, Task, Wrap, DEFAULT_BLOCK, DEFAULT_DAEMON, DEFAULT_MULTIPROC, DEFAULT_PULLEMPTY, DEFAULT_PUSHFULL, DEFAULT_WRAP
from lazy_pyros_utils.ioproc import enum_to_human_str, generate_dir_name, generate_output_str, get_input, get_input_from_sql, sanitise_enum_options, sanitise_non_neg_int, sanitise_not_null, sanitise_with_default, sanitise_with_null, sanitise_yes_no, show_enum_as_options, throw_tantrum

if __name__ == "__main__":
     parser = argparse.ArgumentParser()
     parser.add_argument("path_to_db", help="Path to SQLite database file to be opened")
     db_path = parser.parse_args().path_to_db

     if not os.path.exists(db_path):
          raise Exception(str(db_path)+" does not exist. Create a database using db_create.py first.")
     
     conn = sqlite3.connect(db_path)
     
     # for testing
     # conn = sqlite3.connect("/mnt/c/Users/UserAdmin/Downloads/main.db")

     # https://docs.python.org/3.8/library/sqlite3.html#controlling-transactions
     # put update/delete/etc. statements you want as a single transaction together
     # or commit in between when you want them to be separate transactions
     # sqlite3 (probably) auto-commits before any read (SELECT) statement
     conn.execute("PRAGMA foreign_keys = 1")
     cursor = conn.cursor()

     while True:
          curr_task = get_input("What do you want to do?", "Choosing Your Task", 
                                show_enum_as_options, (Task,), sanitise_enum_options, (Task,))
          print("You chose:", enum_to_human_str(curr_task.name))
          print()

          if curr_task == Task.INSERT_TYPE:
               while True:
                    print("Define New Node Type")
                    type_name = get_input("Name:", sanitise=sanitise_not_null)
                    type_desc = get_input("Description (optional):")
                    num_topic = get_input("Number of published topics:", sanitise=sanitise_non_neg_int)

                    out_topic_values = []
                    while_i = 0
                    while while_i < num_topic:
                         print("    Define Publishing Topic "+str(while_i+1)+"/"+str(num_topic))
                         if get_input("Does the topic already exist? [y/n]", sanitise=sanitise_yes_no) == 'n':
                              name = get_input("Name:", sanitise=sanitise_not_null)
                              desc = get_input("Description (optional):")
                              block = get_input("Can Block Publisher:", show_opts=show_enum_as_options, show_opts_args=(Boolean,DEFAULT_BLOCK), 
                                   sanitise=sanitise_with_default, sanitise_args=(Boolean(DEFAULT_BLOCK), sanitise_enum_options, (Boolean,)))
                              out_topic_values.append((name,desc,block))
                         else:
                              try:
                                   topic_id = get_input_from_sql("ID of topic:", cursor, "SELECT * FROM topic", ["ID", "Name", "Description", "Can Block Publisher"])
                              except Exception as err:
                                   throw_tantrum(str(err))
                                   continue
                              out_topic_values.append((topic_id,))
                         while_i += 1
                    
                    print("\nDetails of New Node Type")
                    print("Name:", generate_output_str(type_name))
                    print("Description:", generate_output_str(type_desc))
                    for i in range(num_topic):
                         print("    Topic "+str(i+1)+"/"+str(num_topic))
                         if len(out_topic_values[i]) == 1:
                              print("Existing Topic ID:", generate_output_str(out_topic_values[i][0]))
                         else:
                              print("Name:", generate_output_str(out_topic_values[i][0]))
                              print("Description:", generate_output_str(out_topic_values[i][1]))
                              print("Can Block Publisher:", generate_output_str(out_topic_values[i][2]))
                    if get_input("\nConfirm details? [y/n]", sanitise=sanitise_yes_no) == 'n':
                         print("Task abandoned!")
                         break

                    cursor.execute("INSERT INTO type (name, desc) VALUES (?,?)", 
                                   (type_name, type_desc))
                    type_id = cursor.lastrowid

                    for out_topic in out_topic_values:
                         if len(out_topic) != 1:
                              cursor.execute("INSERT INTO topic (name, desc, block) VALUES (?,?,?)", 
                                                  out_topic)
                              topic_id = cursor.lastrowid
                         else:
                              topic_id = out_topic[0]
                         cursor.execute("INSERT INTO type_publish_topic (type_id, topic_id) VALUES (?,?)", 
                                             (type_id, topic_id))

                    directory_name = generate_dir_name("type", type_id, type_name)
                    try:
                         os.mkdir(directory_name)
                    except Exception as e:
                         print(f"An error occurred: {e}")
                         conn.rollback()
                         print(f"If no one else is accessing the database, no changes have been made.")
                         break

                    conn.commit()
                    print("\nNew node type created!")
                    break

          elif curr_task == Task.INSERT_NODE:
               while True:
                    print("Define New Node")

                    try:
                         type_id = get_input_from_sql("Node Type ID:", cursor, "SELECT * FROM type", ["ID", "Name", "Description"])
                    except Exception as err:
                         print("Make sure that you have defined at least one node type first.")
                         throw_tantrum(str(err))
                         break

                    node_name = get_input("Name:", sanitise=sanitise_not_null)
                    node_desc = get_input("Description (optional):")
                    pkg_name = get_input("Python package name (optional, if the function you need is in __init__.py):")
                    func_name = get_input("Target function name:", sanitise=sanitise_not_null)

                    wrap = get_input("Wrapper required:", show_opts=show_enum_as_options, show_opts_args=(Wrap,DEFAULT_WRAP), 
                                   sanitise=sanitise_with_default, sanitise_args=(Wrap(DEFAULT_WRAP), sanitise_enum_options, (Wrap,)))
                    if wrap == Wrap.CLASS:
                         method = get_input("Name of class method:", sanitise=sanitise_not_null)
                    else:
                         method = None
                    
                    daemon = get_input("Run thread/process as daemon:", show_opts=show_enum_as_options, show_opts_args=(Boolean,DEFAULT_DAEMON), 
                                   sanitise=sanitise_with_default, sanitise_args=(Boolean(DEFAULT_DAEMON), sanitise_enum_options, (Boolean,)))

                    # TODO: no support for multiprocessing yet
                    # multiproc = get_input("Use multiprocessing:", show_opts=show_enum_as_options, show_opts_args=(Boolean,DEFAULT_MULTIPROC), 
                    #                sanitise=sanitise_with_default, sanitise_args=(Boolean(DEFAULT_MULTIPROC), sanitise_enum_options, (Boolean,)))
                    multiproc = Boolean(DEFAULT_MULTIPROC)

                    if get_input("\nCan the target function run from any working directory? (e.g. it uses relative imports and relative file paths) [y/n]", sanitise=sanitise_yes_no) == 'n':
                         workdir = get_input("Working directory (leave blank if run in top directory):",
                                             sanitise=sanitise_with_default, sanitise_args=("", sanitise_not_null))
                    else:
                         workdir = None

                    in_topic_values = []
                    num_topic = get_input("Number of topic subscriptions:", sanitise=sanitise_non_neg_int)
                    for i in range(num_topic):
                         print("    Define Topic Subscription "+str(i+1)+"/"+str(num_topic))
                         try:
                              topic_id = get_input_from_sql("Topic ID:", cursor, "SELECT id, name, desc, block FROM topic WHERE id NOT IN (SELECT topic_id FROM type_publish_topic WHERE type_id=?)", 
                                                            ["ID", "Name", "Description", "Can Block Publisher"], (type_id,))
                         except Exception as err:
                              num_topic = 0
                              print("Make sure that you have defined at least one topic for another node type.")
                              throw_tantrum(str(err))
                              break
                         size = get_input("Maximum size (optional):", sanitise=sanitise_with_null, sanitise_args=(sanitise_non_neg_int,))
                         if size:
                              cursor.execute("SELECT block FROM topic WHERE id=?", (topic_id,))
                              can_block = cursor.fetchall()[0][0]
                              if can_block == Boolean.TRUE_: display_enum = PushFullDisplayBlock
                              else: display_enum = PushFullDisplayNoBlock
                              pushfull = get_input("Behaviour when pushing to full topic:", show_opts=show_enum_as_options, show_opts_args=(display_enum, DEFAULT_PUSHFULL),
                                                  sanitise=sanitise_with_default, sanitise_args=(display_enum(DEFAULT_PUSHFULL), sanitise_enum_options, (display_enum,)))
                         else:
                              pushfull = PushFull.NO_MAX_SIZE
                         pullempty = get_input("Behaviour when pulling from empty topic:", show_opts=show_enum_as_options, show_opts_args=(PullEmpty, DEFAULT_PULLEMPTY),
                                               sanitise=sanitise_with_default, sanitise_args=(PullEmpty(DEFAULT_PULLEMPTY), sanitise_enum_options, (PullEmpty,)))
                         in_topic_values.append((topic_id, size, pushfull, pullempty))
                    
                    print("\nDetails of New Node")
                    print("Type ID:",generate_output_str(type_id))
                    print("Name:", generate_output_str(node_name))
                    print("Description:", generate_output_str(node_desc))
                    print("Package name:",generate_output_str(pkg_name))
                    print("Target function name:", generate_output_str(func_name))
                    print("Wrapper:", generate_output_str(wrap))
                    print("Class method name:", generate_output_str(method))
                    print("Daemon:",generate_output_str(daemon))
                    print("Multiprocessing:", generate_output_str(multiproc))
                    print("Working directory:", generate_output_str(workdir))
                    for i in range(len(in_topic_values)):
                         print("    Subscribed Topic "+str(i+1)+"/"+str(num_topic))
                         print("Topic ID:", generate_output_str(in_topic_values[i][0]))
                         print("Maximum Size:", generate_output_str(in_topic_values[i][1]))
                         print("Pushing to Full Topic:", generate_output_str(in_topic_values[i][2]))
                         print("Pulling from Empty Topic:", generate_output_str(in_topic_values[i][3]))
                    if get_input("\nConfirm details? [y/n]", sanitise=sanitise_yes_no) == 'n':
                         print("Task abandoned!")
                         break

                    cursor.execute("INSERT INTO node (type_id, name, desc, pkg_name, func, wrap, method, daemon, multiproc, workdir) VALUES (?,?,?,?,?,?,?,?,?,?)", 
                                   (type_id, node_name, node_desc, pkg_name, func_name, wrap, method, daemon, multiproc, workdir))
                    node_id = cursor.lastrowid
                    in_topic_values = [(node_id,) + x for x in in_topic_values]
                    cursor.executemany("INSERT INTO node_subscribe_topic (node_id, topic_id, size, pushfull, pullempty) VALUES (?,?,?,?,?)", 
                                        in_topic_values)

                    # sorting out directory structure
                    try:
                         possible_dirs = [x for x in os.listdir() if x.startswith("type"+str(type_id).zfill(2))]
                         if len(possible_dirs) > 1:
                              raise Exception("Found duplicate directories for type ID "+str(type_id))
                         elif len(possible_dirs) == 0:
                              raise Exception("No directory for type ID "+str(type_id)+" found")
                    except Exception as e:
                         print(f"An error occurred: {e}")
                         conn.rollback()
                         print(f"If no one else is accessing the database, no changes have been made.")
                         break
                    directory_name = generate_dir_name("node", node_id, node_name)
                    directory_name = os.path.join(possible_dirs[0], directory_name)
                    if get_input("\nAre you going to copy the package instead of using symlinks? [y/n]", sanitise=sanitise_yes_no) == 'y':
                         try:
                              os.mkdir(directory_name)
                         except Exception as e:
                              print(f"An error occurred: {e}")
                              conn.rollback()
                              print(f"If no one else is accessing the database, no changes have been made.")
                              break
                         print("\nNew node created!")
                         print("Please copy the CONTENTS of your package into '"+directory_name+"':")
                         print("\ncp -r /path/to/original/package/* "+directory_name)
                    else:
                         print("\nNew node created!")
                         print("Please link your package to '"+directory_name+"':")
                         print("\nln -sir any/path/to/original/package "+directory_name)
                         print("\nln -si /definitely/correct/from/dst/path/to/original/package "+directory_name)
                    conn.commit()
                    
                    break
          elif curr_task == Task.EDIT_TYPE:
               print("Coming soon...")
          elif curr_task == Task.EDIT_NODE:
               print("Coming soon...")
          elif curr_task == Task.DELETE_TYPE:
               print("Coming soon...")
          elif curr_task == Task.DELETE_NODE:
               print("Coming soon...")

          print()
          if get_input("Any more work to do? [y/n]", sanitise=sanitise_yes_no) == 'n':
               break
     
     conn.commit()
     conn.close()
     print("Bye!")

# cursor.execute("""INSERT INTO type (name)
#                     VALUES (?)""", ("test_type_1",))
# cursor.execute("""INSERT INTO type (name)
#                     VALUES (?)""", ("test_type_2",))
# cursor.execute("""DELETE FROM type WHERE id=?""", (1,))
# cursor.execute("""INSERT INTO type (name)
#                     VALUES (?)""", ("test_type_3",))
# cursor.execute("""INSERT INTO type_queue_out (type_id, name, size)
#                     VALUES (?,?,?)""", (2, "test queue", 1))
# cursor.execute("""INSERT INTO module (type_id, name, func, wrap, daemon)
#                     VALUES (?,?,?,?,?)""", (1, 'test_module', 'test_func', 2, 1))
