# -*- coding: utf-8 -*-
"""Abaqus/CAE plug-in registration entry point."""
from abaqusGui import getAFXApp

from cylinderSurfacePattern_forms import CylinderSurfacePatternProcedure


toolset = getAFXApp().getAFXMainWindow().getPluginToolset()
toolset.registerGuiMenuButton(
    buttonText='Cylinder Surface Pattern|Grooves / packed bodies...',
    object=CylinderSurfacePatternProcedure(toolset),
    kernelInitString='import cylinderSurfacePattern_kernel; '
                     'reload(cylinderSurfacePattern_kernel)',
    applicableModules=('Part', 'Mesh'),
    version='1.6.0',
    author='OpenAI',
    description='Cut a local chocolate-style crossed-groove patch or attach '
                'close-packed '
                'cubes/spheres to a cylindrical or variable-radius revolved '
                'face, including raised grooved layers, parallel liners, '
                'rectangular footprints and multi-layer covers; '
                'then partition and mesh the generated part.')
