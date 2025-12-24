# lazy-pyros

![alt text](https://docs.ros.org/en/jazzy/_images/Topic-SinglePublisherandSingleSubscriber.gif "A ROS2 Topic")

An overly simplified and probably very inaccurate version of how [nodes in ROS 2 run concurrently](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#background), but for only Python programs using Python's built-in `threading`, `multiprocessing` and `queue` libraries.

## Important Files

[`db_create.py`](./db_create.py) creates the database using pre-defined parameters.

[`db_manage.py`](./db_manage.py) allows the user to insert new node types and nodes. For other operations, [view or edit the database directly](https://sqlitebrowser.org/).

[`cfg.py`](./cfg.py) is a sample file containing constants shared across all nodes.

[`run.py`](./run.py) is the main driver program, which takes in user-defined nodes and sets up necessary queues and threads.

## Target Function Requirements

1. The target function only requires the following arguments: `topics_in`, `topics_out`
   - Wrapped functions/class methods must have input arguments which correspond to `topics_in`, and outputs that correspond to `topics_out`
2. If the target function uses `asyncio`, wrap it using `asyncio.run()` to create another function as the true target function

### Class Wrapper

1. `__init__` must not require any arguments
2. The target function is the class method `update()`

### Recommended

1. If you define any constants, put them in [`cfg.py`](./cfg.py), which is shared with all other nodes. Import them in your files, regardless of which sub-directory the file is in, using `import cfg`, `from cfg import CONST_1, CONST_2` or [`from cfg import *`](https://docs.python.org/3/faq/programming.html#what-are-the-best-practices-for-using-import-in-a-module)
2. Use [relative imports](https://realpython.com/absolute-vs-relative-python-imports/), and file paths should be relative to the `.py` file, and not the working directory (which is default behaviour)
3. Import the target function in `__init__.py` at the top-level of your package

## Important Reminders

If anything suddenly stops working, it probably is because of `threading`.

1. [Manually initialise context within the thread for PyCUDA.](https://stackoverflow.com/a/50920361) Do it only ONCE per thread.
2. [`pyros-rookMQTT`](https://github.com/DvdrepoMain/Simple-CV/tree/pyros-rookMQTT) now uses symlinks instead of copies of `lazy-pyros` scripts, so that any updates can be applied universally. However, this means that [the symlinked directory is prepended to `PYTHONPATH`](https://docs.python.org/3/library/sys.html#sys.path), so you must ensure that nothing in `lazy-pyros` would affect the imports in `pyros-rookMQTT` (e.g. `cfg.py`).

## Brief Description

### Node

Each node is a running `Thread` or `Process` with a single **target function**.

Its published topics are defined by its **node type**. It can subscribe to specific topics as inputs.

No more than one node per node type should run concurrently.

#### Target Function

The target function handles inputs from subscribed topics, processes them, and publishes outputs to topics. It usually contains a `while True` loop.

Any arguments that are constant should be accessed from [`cfg.py`](./cfg.py) as a constant variable, instead of being passed as an argument. The target function should ONLY require: `topics_in`, `topics_out`. 

`topics_in` is a list of queues, representing the topics that this node subscribed to.

`topics_out` is a list of list of queues, where each sub-list of queues represents one published topic, and each queue represents a single subscriber.

The order of both lists is the order in which they were defined for the database.

#### Node Type

All nodes of the same node type are interchangeable and publish the same topics. However, nodes of the same type can subscribe to different topics.

### Topic

Different node types can publish the same topic, but they should not run concurrently. Each topic can be one-to-many, which is implemented using many one-to-one unidirectional queues. All queues that belong to a topic have the methods: `push()` and `pull()`.

## Limitations

1. All code must run in Python.
2. All code must run in the same environment, without conflicting dependencies. If not, maybe use Docker or something else.
3. `threading` does not actually use multiple CPU threads [(yet)](https://peps.python.org/pep-0703/).
   - Third party libraries in C, such as OpenCV, may [use multiple threads internally and circumvent Python’s GIL](https://peps.python.org/pep-0703/#internal-parallelization).
   - `multiprocessing` uses multiple CPU threads, but has to serialise any data passed between processes, incurring a high overhead cost.
  
### Implementation of Queues

The queues implemented in [`topic.py`](./lazy_pyros_utils/topic.py) tinker with the innards of Python's own `queue.Queue`, inheriting or overriding undocumented and private methods. Therefore, it may not work in future Python versions. This was developed on Python 3.8, and it looks to be compatible up to at least Python 3.14. If it breaks and you want to fix it, refer to [the source of Python's `queue` library](https://github.com/python/cpython/blob/main/Lib/queue.py).

## To-Dos

- [ ] Exit all threads gracefully, elegantly, safely, which PyCUDA requires.
- [ ] Main thread tries to resuscitate dead threads.
- [x] Lock when get() + put() when keeping latest in queue.
- [x] Each topic should no longer be exclusive to a node type.
- [x] Properties of each subscription should be defined by the consumer and not the producer.
- [x] Implement all types of queues (FIFO/LIFO, keep earliest/latest/block on put if full, block on get if empty, multiprocessing queue).
   - LIFO: Pointless?
- [x] [Define constants centrally and pass them to all nodes.](https://docs.python.org/3/faq/programming.html#how-do-i-share-global-variables-across-modules)
   - ~~Create symlink for `cfg.py` when working directory is changed for compatibility~~ Works without this (for now)
- [x] At least one thread should not be daemonic, or let the main thread do stuff (e.g. check thread status) to not exit.
- [x] Let user define class method name in class wrapper.

### The Backburner

- [ ] Multiprocessing queues
- [ ] Manage and merge multiple `requirements.txt`.
- [ ] Many-to-many topics.
- [ ] Nodes can indicate preferences to subscribe to one out of multiple potential topics, so that only one of multiple node types needs to be present, rather than one specific node type.
   - Let each topic be used by multiple node types instead.
- [ ] Define and pass constant arguments into target functions.
   - Arguments that are constants should use centrally defined constants directly, rather than passing it as arguments.

### Database Management

- [x] Code clean-up for better code reuse. (kind of accomplished?)
- [x] Implement comprehensive input sanity checks. (it's ok except for import and file paths)
- [ ] Ensure integrity of directory structure, that it is consistent with the database. (partial, ensures type directory integrity when adding new nodes)
   - All typeXX and nodeXX directories are unique
   - Defined package names and function imports are valid
- [ ] GUI
