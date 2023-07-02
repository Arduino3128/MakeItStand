import bpy, bmesh
import numpy as np
import os
import tempfile
from threading import Thread
import csv
import time
import mathutils
from mathutils import Vector
from collections import deque

done = False


def calculation_engine():
    global com_req, com_curr_arr, ob, done

    com_req = [0, 0, 0]  # X Y Z
    com_curr_arr = [0, 0, 0]
    curr_tolerance = base_tolerance = 0.1
    shrink_factor = -3
    convergence_flag = False
    convergence_buffer_size = 100
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

        context.view_layer.objects.active = mass_body
        bpy.ops.object.mode_set(mode="EDIT")
        mass_body.select_set(True)
        bpy.ops.transform.shrink_fatten(value=shrink_factor)

        modifier = mass_body.modifiers.new(name="Remesh", type="REMESH")
        modifier = bpy.data.objects["MassBody"].modifiers["Remesh"]
        modifier.mode = "BLOCKS"
        modifier.octree_depth = 5
        #modifier.use_remove_disconnected = False

        context.view_layer.objects.active = mass_body
        """remove_mass_body = mass_body.copy()
        remove_mass_body.data = mass_body.data.copy()
        remove_mass_body.name = "RemovedMassBody"
        context.collection.objects.link(remove_mass_body)
        mass_body.select_set(False)
        context.view_layer.objects.active = remove_mass_body
        bpy.ops.transform.shrink_fatten(value=shrink_factor)
        context.view_layer.objects.active = mass_body"""

        mass_body.select_set(True)
        bpy.ops.mesh.select_all(action="DESELECT")
        bpy.ops.mesh.select_mode(type="VERT")

        ob = context.object
        me = ob.data
        bm = bmesh.from_edit_mesh(me)

        def undo():
            print("Undo")
            bpy.ops.ed.undo()

        def create_merge(
            object1_instance, object2_instance, apply=False
        ):  # Merge Object 2 in Object 1, Object 1 becomes parent
            # print("Merging Objects")
            bpy.ops.object.select_all(action="DESELECT")
            context.view_layer.objects.active = object1_instance
            modifier = object1_instance.modifiers.new(name="Boolean", type="BOOLEAN")
            modifier = bpy.data.objects[object1_instance.name].modifiers["Boolean"]
            modifier.operation = "DIFFERENCE"  # "UNION"
            modifier.solver = "FAST"
            modifier.object = object2_instance
            if apply:
                bpy.ops.object.modifier_apply(modifier="Boolean")

        def com_curr():  # TODO: COPY OBJECTS, CREATE AND APPLY MERGE, GET COM
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
            bpy.ops.object.origin_set(type="ORIGIN_CENTER_OF_VOLUME")
            com_curr_temp = bpy.context.object.location
            bpy.ops.ed.undo_push(message="Calculated COM Volume")

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
            com_curr_arr = com_curr_temp
            print(f"COM Currently at: {com_curr_temp}")
            return com_curr_temp

        for axis in range(3):
            print(f"Current Axis: {axis+1}")
            curr_tolerance = base_tolerance
            index_buffer = deque([0] * convergence_buffer_size)
            while not (
                (com_req[axis] + curr_tolerance)
                >= (com_diff := com_curr()[axis])
                >= (com_req[axis] - curr_tolerance)
            ):  # and not convergence_flag
                com_diff -= com_req[axis]
                bpy.ops.object.mode_set(mode="OBJECT")
                bpy.ops.object.mode_set(mode="EDIT")
                ob = context.object
                me = ob.data
                bm = bmesh.from_edit_mesh(me)
                bm.normal_update()
                bm.verts.ensure_lookup_table()
                vs_co = 0
                vs_index = 0
                for vs in bm.verts:
                    if vs.is_valid:
                        global_vs_co = vs.co #ob.matrix_world @ vs.co
                        if com_diff > 0:
                            if global_vs_co[axis] < vs_co:
                                vs_obj = vs
                                vs_co = global_vs_co[axis]
                                vs_index = vs.index
                        else:
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
                        f"\nFAILED\nCould not converge within tolerance in Axis {axis+1}\nTry Increasing the Tolerance!"
                    )
                    break
                """
                if com_diff > 0:
                        if vs_co < com_req[axis]:
                            vs_obj.select_set(True)
                            print(
                                f"Removing Vertex at Axis {axis+1}: {vs_co} at Index: {vs_index}"
                            )
                            bpy.ops.mesh.delete(type="VERT")
                    else:
                        if vs_co > com_req[axis]:
                            vs_obj.select_set(True)
                            print(
                                f"Removing Vertex at Axis {axis+1}: {vs_co} at Index: {vs_index}"
                            )
                            bpy.ops.mesh.delete(type="VERT")"""
                try:
                    vs_obj.select_set(True)
                    print(
                        f"Removing Vertex at Axis {axis+1}: {vs_co} at Index: {vs_index}"
                    )
                    bpy.ops.mesh.delete(type="VERT")
                    bmesh.update_edit_mesh(ob.data)
                except Exception as ERROR:
                    print(f"ERROR: {ERROR}")

        bpy.ops.object.mode_set(mode="OBJECT")
        bpy.ops.object.origin_set(type="ORIGIN_CENTER_OF_VOLUME")
        com_curr_fin = np.array(ob.location)
        print(f"\nFINISHED\nCOM Currently at: {com_curr_fin}")

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
        bpy.ops.object.origin_set(type="ORIGIN_CENTER_OF_VOLUME")
        com_curr_stop = np.array(ob.location)
        print(f"\nSTOPPED\nCOM Currently at: {com_curr_stop}")
    finally:
        done = True


def plot_graph():
    global com_req, com_curr_arr, done
    file_path = tempfile.gettempdir() + "\\MIS.csv"
    iter = 0
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
    # os.system(f"C:\Program Files\Microsoft Office\root\Office16\EXCEL.EXE {file_path}")


if __name__ == "__main__":
    GraphingThread = Thread(target=plot_graph)
    # GraphingThread.start()
    calculation_engine()
    # GraphingThread.join()
