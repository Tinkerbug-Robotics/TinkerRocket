"""Embed out/mold_geometry.json into fusion_template.py -> out/fusion/ReflowMold/ReflowMold.py"""
import json, os
geom = open('out/mold_geometry.json').read()
script = open('fusion_template.py').read().replace("__GEOM_JSON__", geom)
os.makedirs('out/fusion/ReflowMold', exist_ok=True)
open('out/fusion/ReflowMold/ReflowMold.py', 'w').write(script)
open('out/fusion/ReflowMold/ReflowMold.manifest', 'w').write(json.dumps({
    "autodeskProduct": "Fusion360", "type": "script", "author": "Christian Pedersen",
    "description": "Builds the rocket-computer-mini v1.01 reflow-support silicone mold (Mold Max 60) as a parametric Fusion design",
    "supportedOS": "windows|mac", "editEnabled": True}, indent=2))
print("wrote out/fusion/ReflowMold/ReflowMold.py")
