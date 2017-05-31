# robot_core
Contains all the launch & config files of the Jaguarbot. The *robot_core* package is catergorise in the following hierarchy:
- config: Configuration files for the launch files.
- maps: Map files (*.pgm* & *.yaml*)
- launch: Contains the *launch* files
  - archive: Obselete/Experimental launch files
  - basic: Consists only of the most fundamental nodes
  - mapping: For map generation (Run *rosrun map_server map_saver -f [mapname]* to save the map generated.)
  - 
