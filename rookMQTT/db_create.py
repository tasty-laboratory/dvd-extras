import argparse
import os.path
import sqlite3

if __name__ == "__main__":
     parser = argparse.ArgumentParser()
     parser.add_argument("path_to_db", help="Path to SQLite database file to be created")
     db_path = parser.parse_args().path_to_db

     if os.path.exists(db_path):
          raise Exception(str(db_path)+" already exists. Specify a different path.")
     
     conn = sqlite3.connect(db_path)
     
     # for testing
     # conn = sqlite3.connect("/mnt/c/Users/UserAdmin/Downloads/main.db")

     conn.execute("PRAGMA foreign_keys = 1")
     cursor = conn.cursor()

     # AUTOINCREMENT prevents id from being reused
     cursor.execute("""CREATE TABLE type (
                         id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
                         name TEXT NOT NULL,
                         desc TEXT 
                    );""")

     cursor.execute("""CREATE TABLE topic (
                         id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
                         name TEXT NOT NULL,
                         desc TEXT,
                         block INTEGER NOT NULL ON CONFLICT REPLACE DEFAULT 1,
                         CHECK (block IN (0,1))
                    );""")
     
     cursor.execute("""CREATE TABLE type_publish_topic (
                         id INTEGER PRIMARY KEY NOT NULL,
                         type_id INTEGER NOT NULL,
                         topic_id INTEGER NOT NULL,
                         FOREIGN KEY(type_id) REFERENCES type(id),
                         FOREIGN KEY(topic_id) REFERENCES topic(id)
                    );""")
     
     cursor.execute("""CREATE TABLE node (
                         id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
                         type_id INTEGER NOT NULL,
                         name TEXT NOT NULL,
                         desc TEXT,
                         pkg_name TEXT,
                         func TEXT NOT NULL,
                         wrap INTEGER NOT NULL ON CONFLICT REPLACE DEFAULT 0,
                         method TEXT,
                         daemon INTEGER NOT NULL ON CONFLICT REPLACE DEFAULT 1,
                         multiproc INTEGER NOT NULL ON CONFLICT REPLACE DEFAULT 0,
                         workdir TEXT,
                         FOREIGN KEY(type_id) REFERENCES type(id),
                         CHECK (wrap IN (0,1,2) and daemon IN (0,1) and multiproc IN (0,1))
                    );""")

     cursor.execute("""CREATE TABLE node_subscribe_topic (
                         id INTEGER PRIMARY KEY NOT NULL,
                         node_id INTEGER NOT NULL,
                         topic_id INTEGER NOT NULL,
                         size INTEGER,
                         pushfull INTEGER NOT NULL ON CONFLICT REPLACE DEFAULT 2,
                         pullempty INTEGER NOT NULL ON CONFLICT REPLACE DEFAULT 1,
                         FOREIGN KEY(node_id) REFERENCES node(id),
                         FOREIGN KEY(topic_id) REFERENCES topic(id)
                         CHECK (pushfull IN (0,1,2,3) and pullempty IN (0,1))
                    );""")

     conn.commit()
     conn.close()