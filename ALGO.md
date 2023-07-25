### ALGORITHM  
  
  
1. Set Target position of COM.

2. Calculate COM based on Volume of the object --> `MainBody`.
  
3. Create copy of the object --> `MassBody`, which would be the mass component of the final object.

4. Shrink the copied object with a factor, `shrink_factor`.

5. Convert the `MassBody` to voxel grid. Each vertex representing a voxel.

6. Starting with Axis 1 (`X` - Axis), based on the position of current COM and target COM, Vertex/Voxel (of `MassBody`) farthest from the current axis is removed, __Removal of vertex is considered as removal of empty space or addition of mass.__

7. The `MassBody` is combined with `MainBody` and the COM is calculated.

8. With the updated COM, if the COM is within the `base_tolerance`, next Axis is selected. If the COM is not within the `base_tolerance`, next farthest Vertex/Voxel is removed until the COM is within the `base_tolerance`.

9. Simultaneously, the convergence graph is plotted. 

10. If the current COM cannot converge with the target COM for `convergence_buffer_size` iterations, Error is returned and next axis is selected assuming the solution is not possible and the COM (on that axis) just before the error is the best closest solution.