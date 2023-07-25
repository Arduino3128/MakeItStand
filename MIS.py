import bpy, bmesh
import numpy as np
import os
import tempfile
from threading import Thread
import csv
import time
from timeit import default_timer as timer
from math import radians
import mathutils
from collections import deque

done = False
com_req = [0, 0, 0]  # X Y Z
com_curr_arr = (0, 0, 0)
axes = ['X','Y','Z']
iter = 0
inf = float('inf')


def calculation_engine():
    global com_req, com_curr_arr, ob, done,iter
    start = time.time()
    com_req = [0, 0, 0]  # X Y Z
    com_curr_arr = (0, 0, 0)
    curr_tolerance = base_tolerance = 0.05
    com_sample = 10
    shrink_factor = -0.15
    convergence_flag = False
    convergence_buffer_size = 30
    index_buffer = deque([0] * convergence_buffer_size)

    try:
        done = False
        context = bpy.context
        com_req = bpy.context.scene.cursor.location

        def create_body(name):
            active_body = bpy.context.active_object
            dupl_object = active_body.copy()
            dupl_object.data = active_body.data.copy()
            dupl_object.name = "MainBody"
            bpy.context.collection.objects.link(dupl_object)
            dupl_object.select_set(False)
            active_body.name = name
            bpy.context.view_layer.objects.active = active_body
            return (active_body, dupl_object)

        mass_body, main_body = create_body("MassBody")
        bpy.ops.object.mode_set(mode="EDIT")
        mass_body.select_set(True)
        bpy.ops.transform.shrink_fatten(value=shrink_factor)
        bpy.ops.object.mode_set(mode="OBJECT")
        context.view_layer.objects.active = mass_body
        
        modifier = mass_body.modifiers.new(name="Remesh", type="REMESH")
        modifier = bpy.data.objects["MassBody"].modifiers["Remesh"]
        modifier.mode = "BLOCKS"
        modifier.octree_depth = 6
        #modifier.use_remove_disconnected = False
        #bpy.ops.object.modifier_apply(modifier="Remesh")
        
        bpy.ops.object.mode_set(mode="EDIT")
        context.view_layer.objects.active = mass_body

        mass_body.select_set(True)
        bpy.ops.mesh.select_all(action="DESELECT")
        bpy.ops.mesh.select_mode(type="VERT")


        ob = context.object
        me = ob.data
        bm = bmesh.from_edit_mesh(me)
        
        def create_merge(
            object1_instance, object2_instance, apply=False
        ):  # Merge Object 2 in Object 1, Object 1 becomes parent
            bpy.ops.object.select_all(action="DESELECT")
            context.view_layer.objects.active = object1_instance
            modifier = object1_instance.modifiers.new(name="Boolean", type="BOOLEAN")
            modifier = bpy.data.objects[object1_instance.name].modifiers["Boolean"]
            modifier.operation = "DIFFERENCE"  # "UNION"
            modifier.solver = "FAST"
            modifier.object = object2_instance
            if apply:
                bpy.ops.object.modifier_apply(modifier="Boolean")

        def com_curr():
            global ob, me, bm, com_curr_arr
            bpy.ops.object.mode_set(mode="OBJECT")

            # Set the active object as mass_body and duplicate it
            simulated_mass_body = mass_body.copy()
            simulated_mass_body.data = mass_body.data.copy()
            simulated_mass_body.name = "SimulatedMassBody"
            bpy.context.collection.objects.link(simulated_mass_body)
            simulated_mass_body.select_set(False)

            # Set the active object as main_body and duplicate it
            simulated_main_body = main_body.copy()
            simulated_main_body.data = main_body.data.copy()
            simulated_main_body.name = "SimulatedMainBody"
            bpy.context.collection.objects.link(simulated_main_body)
            simulated_main_body.select_set(False)

            create_merge(simulated_main_body, simulated_mass_body, apply=True)

            bpy.ops.object.select_all(action="DESELECT")
            simulated_main_body.select_set(True)
            context.view_layer.objects.active = simulated_main_body
            vector_2 =np.array([0.0,0.0,0.0])
            for i in range(com_sample):
                bpy.ops.object.origin_set(type="ORIGIN_CENTER_OF_VOLUME")
                # print(f"LOCATION: {bpy.context.object.matrix_world.translation}")
                # print(f"WORLD: {bpy.context.object.location}")
                vector_1 = np.array(bpy.context.object.location)
                vector_2 += vector_1
                
            com_curr_temp = mathutils.Vector(np.divide(vector_2,com_sample))
            #bpy.ops.ed.undo_push(message="Calculated COM Volume")

            simulated_main_body.select_set(True)
            simulated_mass_body.select_set(True)
            bpy.ops.object.delete()

            context.view_layer.objects.active = mass_body
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.select_all(action="DESELECT")
            bpy.ops.mesh.select_mode(type="VERT")
            ob = context.object
            me = ob.data
            bm = bmesh.from_edit_mesh(me)
            com_curr_arr = tuple(com_curr_temp)
            print(f"COM Currently at: {com_curr_arr}")
            return com_curr_temp
        
        for axis in range(3):
            print(f"Current Axis: {axis+1}")
            curr_tolerance = base_tolerance
            index_buffer = deque([0] * convergence_buffer_size)

            while not ((com_req[axis] + curr_tolerance)>= (com_diff := com_curr()[axis])>= (com_req[axis] - curr_tolerance)):  # and not convergence_flag
                com_diff -= com_req[axis] # com_curr - com_req
                bpy.ops.object.mode_set(mode="OBJECT")
                bpy.ops.object.mode_set(mode="EDIT")
                ob = context.object
                me = ob.data
                bm = bmesh.from_edit_mesh(me)
                bm.normal_update()
                bm.verts.ensure_lookup_table()
                if com_diff > 0: #if difference is positive, remove the farthest vertex (i.e. smallest) in the negative direction
                    vs_co = inf
                    vs_index = 0
                    for vs in bm.verts:
                        if vs.is_valid:
                            global_vs_co = ob.matrix_world @ vs.co#vs.co
                            if global_vs_co[axis] < vs_co:
                                vs_obj = vs
                                vs_co = global_vs_co[axis]
                                vs_index = vs.index
                        else:
                            break
                        
                else: #if difference is negative, remove the farthest vertex (i.e. largest) in the positive direction
                    vs_co = -inf
                    vs_index = 0
                    for vs in bm.verts:
                        if vs.is_valid:
                            global_vs_co = ob.matrix_world @ vs.co#vs.co
                            if global_vs_co[axis] > vs_co:
                                vs_obj = vs
                                vs_co = global_vs_co[axis]
                                vs_index = vs.index
                        else:
                            break

                index_buffer.popleft()
                index_buffer.append(vs_index)
                if index_buffer.count(vs_index) == convergence_buffer_size:
                    print(
                        f"\nFAILED\nCould not converge within tolerance in {axes[axis]}-Axis\nTry Increasing the Tolerance!"
                    )
                    break
                if not (iter%50):
                    bpy.ops.outliner.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
                
                try:
                    vs_obj.select_set(True)
                    print(
                        f"Removing Vertex in {axes[axis]}-Axis: {vs_co} at Index: {vs_index}",end="\r")
                    bpy.ops.mesh.delete(type="VERT")
                    bmesh.update_edit_mesh(ob.data)
                except Exception as ERROR:
                    print(f"ERROR: {ERROR}")

        bpy.ops.object.mode_set(mode="OBJECT")
        print(f"\nFINISHED\nCOM Currently at: {com_curr_arr}")

        def export_stl():
            context.view_layer.objects.active = main_body
            main_body.select_set(True)
            file_path = tempfile.gettempdir()
            stl_path = file_path + "\\MIS.stl"
            bpy.ops.export_mesh.stl(filepath=str(stl_path), use_selection=True)
            print(f"Exported to {stl_path}")

        def import_stl():
            file_path = tempfile.gettempdir()
            stl_path = file_path + "\\MIS.stl"
            bpy.ops.import_mesh.stl(filepath=str(stl_path))

        create_merge(main_body, mass_body, apply=False)
        export_stl()
        import_stl()

    except KeyboardInterrupt:
        bpy.ops.object.mode_set(mode="OBJECT")
        print(f"\nSTOPPED\nCOM Currently at: {com_curr_arr}")
    finally:
        done = True
        bpy.ops.outliner.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
        elapsed_time=time.time()-start
        print(f'Total Time taken: {time.strftime("%H:%M:%S", time.gmtime(elapsed_time))}')

def plot_graph():
    global com_req, com_curr_arr, done,iter
    file_path = tempfile.gettempdir() + "\\MIS.csv"
    pipe_path = tempfile.gettempdir() + "\\MIS"
    iter = 0
    """try:
        os.mkfifo(pipe_path)
    except OSError, e:
        print(f"Failed to create FIFO: {e}")
    else:
        fifo = open(pipe_path, 'w')"""
    with open(file_path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["ITERATION", "X", "Y", "Z"])
        while not done:
            iter += 1
            curr_com = com_curr_arr
            writer.writerow(
                [iter] + [curr - req for req, curr in zip(com_req, curr_com)]
            )
            time.sleep(0.05)
    print(f"Saved Convergence data to: {file_path}")


if __name__ == "__main__":
    GraphingThread = Thread(target=plot_graph)
    # LiveGraphingThread = Thread(target=plot_live_graph)
    GraphingThread.start()
    # LiveGraphingThread.start()
    calculation_engine()
    #   LiveGraphingThread.join()
    GraphingThread.join()
