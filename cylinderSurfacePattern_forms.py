# -*- coding: utf-8 -*-
"""AFX form/procedure for Abaqus/CAE 2020."""
from abaqusGui import *

from cylinderSurfacePatternDB import CylinderSurfacePatternDB


class CylinderSurfacePatternProcedure(AFXProcedure):

    def __init__(self, owner):
        AFXProcedure.__init__(self, owner)
        self.cmd = AFXGuiCommand(
            self, 'createCylinderSurfacePattern',
            'cylinderSurfacePattern_kernel')

        self.modelNameKw = AFXStringKeyword(
            self.cmd, 'modelName', True, 'Model-1')
        self.sourcePartNameKw = AFXStringKeyword(
            self.cmd, 'sourcePartName', True, 'Part-1')
        self.outputPartNameKw = AFXStringKeyword(
            self.cmd, 'outputPartName', True, 'CylinderSurfacePattern')
        self.operationKw = AFXIntKeyword(
            self.cmd, 'operation', True, 0, evalExpression=False)

        self.grooveRowsKw = AFXIntKeyword(
            self.cmd, 'grooveRows', True, 5)
        self.grooveColumnsKw = AFXIntKeyword(
            self.cmd, 'grooveColumns', True, 4)
        self.grooveAxialLengthKw = AFXFloatKeyword(
            self.cmd, 'grooveAxialLength', True, 0.8)
        self.grooveCircumWidthKw = AFXFloatKeyword(
            self.cmd, 'grooveCircumWidth', True, 0.8)
        self.grooveDepthKw = AFXFloatKeyword(
            self.cmd, 'grooveDepth', True, 1.0)
        self.grooveBlockAxialLengthKw = AFXFloatKeyword(
            self.cmd, 'grooveBlockAxialLength', True, 2.0)
        self.grooveBlockCircumWidthKw = AFXFloatKeyword(
            self.cmd, 'grooveBlockCircumWidth', True, 2.0)
        self.grooveAxialOffsetKw = AFXFloatKeyword(
            self.cmd, 'grooveAxialOffset', True, 0.0)
        self.grooveAngularOffsetKw = AFXFloatKeyword(
            self.cmd, 'grooveAngularOffset', True, 0.0)
        self.fitGroovePatchToFaceKw = AFXBoolKeyword(
            self.cmd, 'fitGroovePatchToFace',
            AFXBoolKeyword.TRUE_FALSE, True, True)
        self.grooveEndMarginKw = AFXFloatKeyword(
            self.cmd, 'grooveEndMargin', True, 1.0)
        self.grooveConstructionModeKw = AFXIntKeyword(
            self.cmd, 'grooveConstructionMode', True, 0,
            evalExpression=False)
        self.grooveRaisedThicknessKw = AFXFloatKeyword(
            self.cmd, 'grooveRaisedThickness', True, 1.0)
        self.grooveLinerThicknessKw = AFXFloatKeyword(
            self.cmd, 'grooveLinerThickness', True, 0.0)

        self.attachmentRowsKw = AFXIntKeyword(
            self.cmd, 'attachmentRows', True, 5)
        self.attachmentColumnsKw = AFXIntKeyword(
            self.cmd, 'attachmentColumns', True, 4)
        self.attachmentSizeKw = AFXFloatKeyword(
            self.cmd, 'attachmentSize', True, 1.0)
        self.packingKw = AFXIntKeyword(
            self.cmd, 'packing', True, 0, evalExpression=False)
        self.embedFractionKw = AFXFloatKeyword(
            self.cmd, 'embedFraction', True, 0.05)
        self.fitAttachmentsToFaceKw = AFXBoolKeyword(
            self.cmd, 'fitAttachmentsToFace',
            AFXBoolKeyword.TRUE_FALSE, True, False)
        self.detachedAttachmentsKw = AFXBoolKeyword(
            self.cmd, 'detachedAttachments',
            AFXBoolKeyword.TRUE_FALSE, True, False)
        self.attachmentClearanceKw = AFXFloatKeyword(
            self.cmd, 'attachmentClearance', True, 0.1)
        self.attachmentMeridianSizeKw = AFXFloatKeyword(
            self.cmd, 'attachmentMeridianSize', True, 0.0)
        self.attachmentCircumSizeKw = AFXFloatKeyword(
            self.cmd, 'attachmentCircumSize', True, 0.0)
        self.attachmentRowShiftDegreesKw = AFXFloatKeyword(
            self.cmd, 'attachmentRowShiftDegrees', True, 10.0)
        self.attachmentLayersKw = AFXIntKeyword(
            self.cmd, 'attachmentLayers', True, 1)
        self.attachmentLayerPackingKw = AFXIntKeyword(
            self.cmd, 'attachmentLayerPacking', True, 0,
            evalExpression=False)

        self.meshSizeKw = AFXFloatKeyword(
            self.cmd, 'meshSize', True, 1.0)
        self.meshTypeKw = AFXIntKeyword(
            self.cmd, 'meshType', True, 4, evalExpression=False)
        self.elementLibraryKw = AFXIntKeyword(
            self.cmd, 'elementLibrary', True, 0, evalExpression=False)
        self.autoPartitionKw = AFXBoolKeyword(
            self.cmd, 'autoPartition', AFXBoolKeyword.TRUE_FALSE,
            True, True)
        self.overwriteKw = AFXBoolKeyword(
            self.cmd, 'overwrite', AFXBoolKeyword.TRUE_FALSE,
            True, False)

        # Construct viewport-pick keywords after ordinary dialog keywords.
        self.facesKw = AFXObjectKeyword(self.cmd, 'faces', True)
        self.dialogStep = None
        self.pickStep = None

    def getFirstStep(self):
        # AFXProcedure instances registered as plug-ins are reused. Pick and
        # dialog steps are C++ GUI objects that are destroyed when a run
        # closes; retaining either object and returning it on the next run can
        # crash CAE. Always build a fresh step chain and clear the old Face.
        self.dialogStep = None
        self.pickStep = None
        try:
            self.facesKw.setValue(None)
        except Exception:
            pass
        self.cmd.setKeywordValuesToDefaults()
        self.dialogStep = AFXDialogStep(
            self, CylinderSurfacePatternDB(self),
            'Set parameters, then pick one revolved exterior face')
        return self.dialogStep

    def getNextStep(self, previousStep):
        if previousStep == self.dialogStep:
            if self.pickStep is None:
                self.pickStep = AFXPickStep(
                    self, self.facesKw,
                    'Pick exactly one revolved exterior face in Part '
                    'context, then click Done',
                    AFXPickStep.FACES, AFXPickStep.ONE, 1,
                    AFXPickStep.ARRAY)
            return self.pickStep
        return None
