"""
Blender material definitions for dragonfly visualization.

This module is imported by render_dragonfly.py when running inside Blender.
"""

import bpy


def create_body_material(name="DragonBody"):
    """
    Create material for dragonfly body (tan/brown color).

    Returns:
        bpy.types.Material: The created material
    """
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # Clear default nodes
    nodes.clear()

    # Create principled BSDF
    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.location = (0, 0)
    bsdf.inputs['Base Color'].default_value = (0.82, 0.71, 0.55, 1.0)  # tan
    bsdf.inputs['Roughness'].default_value = 0.6
    bsdf.inputs['Specular IOR Level'].default_value = 0.3

    # Create output
    output = nodes.new('ShaderNodeOutputMaterial')
    output.location = (300, 0)

    links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])

    return mat


def create_wing_material(name="DragonWing"):
    """
    Create semi-transparent material for dragonfly wings.

    Returns:
        bpy.types.Material: The created material
    """
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    mat.blend_method = 'BLEND'
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # Clear default nodes
    nodes.clear()

    # Create principled BSDF
    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.location = (0, 0)
    bsdf.inputs['Base Color'].default_value = (0.85, 0.85, 0.85, 1.0)  # light gray
    bsdf.inputs['Alpha'].default_value = 0.8
    bsdf.inputs['Roughness'].default_value = 0.2
    bsdf.inputs['Specular IOR Level'].default_value = 0.5
    bsdf.inputs['Transmission Weight'].default_value = 0.1  # slight transparency

    # Create output
    output = nodes.new('ShaderNodeOutputMaterial')
    output.location = (300, 0)

    links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])

    return mat


def get_or_create_body_material():
    """Get existing body material or create new one."""
    name = "DragonBody"
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    return create_body_material(name)


def get_or_create_wing_material():
    """Get existing wing material or create new one."""
    name = "DragonWing"
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    return create_wing_material(name)
