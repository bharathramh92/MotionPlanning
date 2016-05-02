RRTStarReedsShepp.py is the entry point file and Environments are defined in Env directory.

Programming language: Python
Version: 3.4.3

Requirements for this project are mentioned in requirements.txt file.

To run the program to generate the path, use the following command.
- python3 RRTStarReedsShepp.py <Input_JSON_File>
- eg: python3 RRTStarReedsShepp.py Env/EnvRRTStarRS10.json

To just plot the final graph, use the following command.
- python3 DrawGraph.py <Output_of_Previous_Step>
- eg: python3 DrawGraph.py Env/EnvRRTStarRS10_1000_graph.json
