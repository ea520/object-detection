types = (
"inhalation-hazard",
"infectious-substance",
# "{obj_type}",
"non-flammable-gas",
"organic-peroxide",
"flammable",
"radioactive",
"spontaneously-combustible",
"oxygen",
"dangerous",
"flammable-solid",
"corrosive",
"poison",
)
for obj_type in types:
    txt = f"""
<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <asset>
    <contributor>
      <author>Blender User</author>
      <authoring_tool>Blender 2.82.7</authoring_tool>
    </contributor>
    <created>2022-12-04T12:57:51</created>
    <modified>2022-12-04T12:57:51</modified>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
    <effect id="Material-effect">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <emission>
              <color sid="emission">0 0 0 1</color>
            </emission>
            <diffuse>
              <color sid="diffuse">0.8 0.8 0.8 1</color>
            </diffuse>
            <index_of_refraction>
              <float sid="ior">1.45</float>
            </index_of_refraction>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
    <effect id="Material_001-effect">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <emission>
              <color sid="emission">0 0 0 1</color>
            </emission>
            <diffuse>
              <color sid="diffuse">0.8 0.8 0.8 1</color>
            </diffuse>
            <index_of_refraction>
              <float sid="ior">1.45</float>
            </index_of_refraction>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
    <effect id="Texture-effect">
      <profile_COMMON>
        <newparam sid="{obj_type}_png-surface">
          <surface type="2D">
            <init_from>{obj_type}_png</init_from>
          </surface>
        </newparam>
        <newparam sid="{obj_type}_png-sampler">
          <sampler2D>
            <source>{obj_type}_png-surface</source>
          </sampler2D>
        </newparam>
        <technique sid="common">
          <lambert>
            <emission>
              <color sid="emission">0 0 0 1</color>
            </emission>
            <diffuse>
              <texture texture="{obj_type}_png-sampler" texcoord="UVMap"/>
            </diffuse>
            <index_of_refraction>
              <float sid="ior">1.45</float>
            </index_of_refraction>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_images>
    <image id="{obj_type}_png" name="{obj_type}_png">
      <init_from>{obj_type}.png</init_from>
    </image>
  </library_images>
  <library_materials>
    <material id="Material-material" name="Material">
      <instance_effect url="#Material-effect"/>
    </material>
    <material id="Material_001-material" name="Material.001">
      <instance_effect url="#Material_001-effect"/>
    </material>
    <material id="Texture-material" name="Texture">
      <instance_effect url="#Texture-effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="Cube-mesh" name="Cube">
      <mesh>
        <source id="Cube-mesh-positions">
          <float_array id="Cube-mesh-positions-array" count="24">-1 -1 -1 -1 -1 1 -1 1 -1 -1 1 1 1 -1 -1 1 -1 1 1 1 -1 1 1 1</float_array>
          <technique_common>
            <accessor source="#Cube-mesh-positions-array" count="8" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="Cube-mesh-normals">
          <float_array id="Cube-mesh-normals-array" count="18">-1 0 0 0 1 0 1 0 0 0 -1 0 0 0 -1 0 0 1</float_array>
          <technique_common>
            <accessor source="#Cube-mesh-normals-array" count="6" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="Cube-mesh-map-0">
          <float_array id="Cube-mesh-map-0-array" count="72">0.625 0 0.375 0.25 0.375 0 0.625 0.25 0.375 0.5 0.375 0.25 -0.003427445 0.4992493 0.9963724 0.5007507 0.4974856 0.9998998 0.625 0.75 0.375 1 0.375 0.75 0.375 0.5 0.125 0.75 0.125 0.5 0.875 0.5 0.625 0.75 0.625 0.5 0.625 0 0.625 0.25 0.375 0.25 0.625 0.25 0.625 0.5 0.375 0.5 -0.003427445 0.4992493 0.497223 -0.001663625 0.9963724 0.5007507 0.625 0.75 0.625 1 0.375 1 0.375 0.5 0.375 0.75 0.125 0.75 0.875 0.5 0.875 0.75 0.625 0.75</float_array>
          <technique_common>
            <accessor source="#Cube-mesh-map-0-array" count="36" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="Cube-mesh-vertices">
          <input semantic="POSITION" source="#Cube-mesh-positions"/>
        </vertices>
        <triangles material="Material-material" count="10">
          <input semantic="VERTEX" source="#Cube-mesh-vertices" offset="0"/>
          <input semantic="NORMAL" source="#Cube-mesh-normals" offset="1"/>
          <input semantic="TEXCOORD" source="#Cube-mesh-map-0" offset="2" set="0"/>
          <p>1 0 0 2 0 1 0 0 2 3 1 3 6 1 4 2 1 5 5 3 9 0 3 10 4 3 11 6 4 12 0 4 13 2 4 14 3 5 15 5 5 16 7 5 17 1 0 18 3 0 19 2 0 20 3 1 21 7 1 22 6 1 23 5 3 27 1 3 28 0 3 29 6 4 30 4 4 31 0 4 32 3 5 33 1 5 34 5 5 35</p>
        </triangles>
        <triangles material="Texture-material" count="2">
          <input semantic="VERTEX" source="#Cube-mesh-vertices" offset="0"/>
          <input semantic="NORMAL" source="#Cube-mesh-normals" offset="1"/>
          <input semantic="TEXCOORD" source="#Cube-mesh-map-0" offset="2" set="0"/>
          <p>7 2 6 4 2 7 6 2 8 7 2 24 5 2 25 4 2 26</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="Cube" name="Cube" type="NODE">
        <matrix sid="transform">0.01 0 0 0 0 -0.106066 -0.106066 0 0 0.106066 -0.106066 0 0 0 0 1</matrix>
        <instance_geometry url="#Cube-mesh" name="Cube">
          <bind_material>
            <technique_common>
              <instance_material symbol="Material-material" target="#Material-material">
                <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
              <instance_material symbol="Material_001-material" target="#Material_001-material">
                <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
              <instance_material symbol="Texture-material" target="#Texture-material">
                <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
    """
    open(f"{obj_type}.dae", "w").write(txt)