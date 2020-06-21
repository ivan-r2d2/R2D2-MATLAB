# Maps to perform robot autonomous navigations
There are some grid files in the ROS format (Yaml and Image file)

![Alt text](map_01e.png?raw=true "Title")

In the ".mat" files we have the lines map that were used to
construct the grid maps. 
This line maps are also used by the class "laser" to simulate
laser measurements.

![Alt text](map_01.png?raw=true "Title").

The code to generarte this kind of maps is (Each row is a line):

```Matlab
path = ... % Put the path where the map will be saved
name = ... % Put the name of the map
map = ...
     [ 1.02   1   1.02  10;
       1  10  10  10;
      10  10  10   1;
       1   1  10   1;         % Externo
       2   2   2   4;
       2   4   8   4;
       8   4   8   2;
       2   2   8   2;         % Bloque inferior
       2   6   2  10;
       2   6   4   6;
       4   6   4   8;
       4   8   7   8;
       7   8   7  10;         % Bloque
       5   5  10   5;
       5   5   5   6;
       5   6  10   6];
%  1.2. SAVE THE MAP            
save([path, name, '.mat'], 'map')
```
