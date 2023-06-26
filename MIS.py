import bpy, bmesh
import numpy as np
import os
import tempfile
from threading import Thread
import csv
import time
from collections import deque

done = False


def calculation_engine():
    global com_req, com_curr_arr, ob, done

    com_req = [0, 0, 0]  # X Y Z
    com_curr_arr = [0, 0, 0]
    curr_tolerance = base_tolerance = 0.075
    shrink_factor = -0.1
    convergence_flag = False
    convergence_buffer_size = 20
    index_buffer = deque([0] * convergence_buffer_size)

    try:
        done = False
        context = bpy.context
        com_req = bpy.context.scene.cursor.location

        def create_body(name):
            active_body = context.active_object
            dupl_object = bpy.ops.object.duplicate(linked=False)
            dupl_object = bpy.context.active_object
            dupl_object.name = "MainBody"
            dupl_object.select_set(False)
            context.view_layer.objects.active = active_body
            active_body.name = name
            return active_body, dupl_object

        mass_body, main_body = create_body("MassBody")

        context.view_layer.objects.active = mass_body
        bpy.ops.object.mode_set(mode="EDIT")
        mass_body.select_set(True)
        bpy.ops.transform.shrink_fatten(value=shrink_factor)

        modifier = mass_body.modifiers.new(name="Remesh", type="REMESH")
        modifier = bpy.data.objects["MassBody"].modifiers["Remesh"]
        modifier.mode = "BLOCKS"
        modifier.octree_depth = 5
        modifier.use_remove_disconnected = False

        context.view_layer.objects.active = mass_body
        """remove_mass_body = mass_body.copy()
        remove_mass_body.data = mass_body.data.copy()
        remove_mass_body.name = "RemovedMassBody"
        context.collection.objects.link(remove_mass_body)
        mass_body.select_set(False)
        context.view_layer.objects.active = remove_mass_body
        bpy.ops.transform.shrink_fatten(value=shrink_factor)"""

        context.view_layer.objects.active = mass_body
        mass_body.select_set(True)
        bpy.ops.mesh.select_all(action="DESELECT")
        bpy.ops.mesh.select_mode(type="VERT")

        ob = context.object
        me = ob.data
        bm = bmesh.from_edit_mesh(me)

        def com_curr():
            global ob, me, bm, com_curr_arr
            bpy.ops.object.mode_set(mode="OBJECT")
            bpy.ops.object.origin_set(type="ORIGIN_CENTER_OF_VOLUME")
            com_curr_temp = ob.location
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.select_all(action="DESELECT")
            bpy.ops.mesh.select_mode(type="VERT")
            ob = context.object
            me = ob.data
            bm = bmesh.from_edit_mesh(me)
            com_curr_arr = np.array(com_curr_temp)
            return com_curr_temp

        for axis in range(3):
            print(f"Current Axis: {axis+1}")
            curr_tolerance = base_tolerance
            index_buffer = deque([0] * convergence_buffer_size)
            while not (
                (com_req[axis] + curr_tolerance)
                >= (com_curr()[axis])
                >= (com_req[axis] - curr_tolerance)
            ):  # and not convergence_flag
                com_diff = com_curr()[axis] - com_req[axis]
                ob = context.object
                me = ob.data
                bm = bmesh.from_edit_mesh(me)
                bm.verts.ensure_lookup_table()
                vs_co = 0
                vs_index = 0
                for vs in bm.verts:
                    if com_diff < 0:
                        if vs.co[axis] < vs_co:
                            vs_obj = vs
                            vs_co = vs.co[axis]
                            vs_index = vs.index
                    else:
                        if vs.co[axis] > vs_co:
                            vs_obj = vs
                            vs_co = vs.co[axis]
                            vs_index = vs.index

                index_buffer.popleft()
                index_buffer.append(vs_index)
                if index_buffer.count(vs_index) == convergence_buffer_size:
                    print(
                        f"\nFAILED\nCould not converge within tolerance in Axis {axis+1}\nTry Increasing the Tolerance!"
                    )
                    break

                print(f"COM Currently at: {com_curr_arr}")
                if com_diff < 0:
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
                        bpy.ops.mesh.delete(type="VERT")

        bpy.ops.object.mode_set(mode="OBJECT")
        bpy.ops.object.origin_set(type="ORIGIN_CENTER_OF_VOLUME")
        com_curr = np.array(ob.location)
        print(f"\nFINISHED\nCOM Currently at: {com_curr}")

        def create_merge(apply=False):
            print("Merging Objects")
            bpy.ops.object.select_all(action="DESELECT")
            context.view_layer.objects.active = main_body
            modifier = main_body.modifiers.new(name="Boolean", type="BOOLEAN")
            modifier = bpy.data.objects["MainBody"].modifiers["Boolean"]
            modifier.operation = "DIFFERENCE"
            modifier.object = mass_body
            if apply:
                bpy.ops.object.modifier_apply(modifier="Boolean")

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

        create_merge(apply=False)
        export_stl()
        import_stl()

    except KeyboardInterrupt:
        bpy.ops.object.mode_set(mode="OBJECT")
        bpy.ops.object.origin_set(type="ORIGIN_CENTER_OF_VOLUME")
        com_curr = np.array(ob.location)
        print(f"\nSTOPPED\nCOM Currently at: {com_curr}")
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
            writer.writerow(
                [iter] + [curr - req for req, curr in zip(com_req, com_curr_arr)]
            )
            time.sleep(0.015)
    print(f"Saved Convergence data to: {file_path}")
    # os.system(f"C:\Program Files\Microsoft Office\root\Office16\EXCEL.EXE {file_path}")


if __name__ == "__main__":
    GraphingThread = Thread(target=plot_graph)
    GraphingThread.start()
    calculation_engine()
    GraphingThread.join()
