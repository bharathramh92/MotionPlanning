import sys

from Config import Environment

if __name__ == "__main__":
    with open(sys.argv[1], "r", encoding="utf-8") as in_file:
        # read the json file.
        env = Environment(sys.argv[1])
        path_lines = env.environment["path_lines"]
        tree_lines = env.environment["tree_lines"]
        # plot the graph
        env.draw_env(tree_lines, path_lines)
