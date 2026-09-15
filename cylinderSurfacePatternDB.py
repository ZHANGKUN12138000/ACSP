# -*- coding: utf-8 -*-
"""Compact tabbed FOX Toolkit dialog for Cylinder Surface Pattern."""
from abaqusGui import *


class CylinderSurfacePatternDB(AFXDataDialog):

    def __init__(self, procedure):
        AFXDataDialog.__init__(
            self, procedure, 'Cylinder Surface Pattern',
            self.CONTINUE | self.CANCEL, DIALOG_ACTIONS_SEPARATOR)
        self.procedure = procedure
        self._visibleState = None

        # Keep the dialog usable on smaller screens. The vertical scrollbar
        # is always visible on the right; horizontal scrolling is disabled.
        scroll = FXScrollWindow(
            self,
            VSCROLLER_ALWAYS | HSCROLLING_OFF |
            LAYOUT_FILL_X | LAYOUT_FIX_HEIGHT,
            0, 0, 0, 500)
        main = FXVerticalFrame(
            scroll, LAYOUT_FILL_X | LAYOUT_FILL_Y)
        self.tabs = FXTabBook(
            main, None, 0, LAYOUT_FILL_X | LAYOUT_FILL_Y)

        # Tab 1: identity and operation. The operation keyword drives which
        # of the two parameter tabs is visible in processUpdates().
        FXTabItem(self.tabs, '1  Basic / Operation')
        basic_page = FXVerticalFrame(
            self.tabs, FRAME_RAISED | LAYOUT_FILL_X | LAYOUT_FILL_Y)
        FXLabel(
            basic_page,
            'Creates a new part from one picked revolved exterior face. '
            'The source part is preserved. Select an operation here, then '
            'open the visible settings tab above.',
            None, JUSTIFY_LEFT | LAYOUT_FILL_X)
        identity = FXGroupBox(
            basic_page, 'Source and operation',
            FRAME_GROOVE | LAYOUT_FILL_X)
        identity_matrix = FXMatrix(
            identity, 2, MATRIX_BY_COLUMNS | LAYOUT_FILL_X)
        AFXTextField(identity_matrix, 18, 'Model:',
                     procedure.modelNameKw, 0)
        AFXTextField(identity_matrix, 18, 'Source part:',
                     procedure.sourcePartNameKw, 0)
        operation = AFXComboBox(
            identity_matrix, 34, 3, 'Operation:', procedure.operationKw, 0)
        operation.appendItem('Cut chocolate-style crossed grooves', 0)
        operation.appendItem('Attach / cover with cubes', 1)
        operation.appendItem('Attach / cover with spheres', 2)

        # Tab 2A: groove-only parameters. This whole tab is hidden for cube
        # and sphere operations so the dialog never shows irrelevant fields.
        self.grooveTab = FXTabItem(self.tabs, '2  Groove Settings')
        self.groovePage = FXVerticalFrame(
            self.tabs, FRAME_RAISED | LAYOUT_FILL_X | LAYOUT_FILL_Y)
        FXLabel(
            self.groovePage,
            'Rows/columns count the raised curved blocks. Fit-to-face '
            'calculates uniform block sizes after reserving both end margins.',
            None, JUSTIFY_LEFT | LAYOUT_FILL_X)
        FXCheckButton(
            self.groovePage,
            'Fit blocks + grooves exactly to the picked face (default)',
            procedure.fitGroovePatchToFaceKw, 0)
        groove = FXGroupBox(
            self.groovePage, 'Chocolate crossed grooves',
            FRAME_GROOVE | LAYOUT_FILL_X)
        groove_matrix = FXMatrix(
            groove, 2, MATRIX_BY_COLUMNS | LAYOUT_FILL_X)
        construction = AFXComboBox(
            groove_matrix, 34, 2, 'Groove construction:',
            procedure.grooveConstructionModeKw, 0)
        construction.appendItem('Cut grooves into source body', 0)
        construction.appendItem('Add raised grooved blocks; keep base intact',
                                1)
        AFXTextField(groove_matrix, 12, 'Raised-block rows:',
                     procedure.grooveRowsKw, 0)
        AFXTextField(groove_matrix, 12, 'Raised-block columns:',
                     procedure.grooveColumnsKw, 0)
        self.grooveDepthField = AFXTextField(
            groove_matrix, 12, 'Groove depth into outer surface:',
            procedure.grooveDepthKw, 0)
        self.grooveRaisedThicknessField = AFXTextField(
            groove_matrix, 12, 'Raised grooved-block thickness:',
            procedure.grooveRaisedThicknessKw, 0)
        AFXTextField(groove_matrix, 12,
                     'Parallel outer liner thickness (0 = none):',
                     procedure.grooveLinerThicknessKw, 0)
        AFXTextField(groove_matrix, 12,
                     'Un-grooved margin at each meridian end:',
                     procedure.grooveEndMarginKw, 0)
        self.manualGrooveFields = []
        self.manualGrooveFields.append(AFXTextField(
            groove_matrix, 12, 'Block meridian length (manual mode):',
            procedure.grooveBlockAxialLengthKw, 0))
        self.manualGrooveFields.append(AFXTextField(
            groove_matrix, 12,
            'Block circumferential width (manual mode):',
            procedure.grooveBlockCircumWidthKw, 0))
        AFXTextField(groove_matrix, 12,
                     'Transverse groove width (meridian):',
                     procedure.grooveAxialLengthKw, 0)
        AFXTextField(groove_matrix, 12,
                     'Longitudinal groove width (arc):',
                     procedure.grooveCircumWidthKw, 0)
        self.manualGrooveFields.append(AFXTextField(
            groove_matrix, 12, 'Patch meridian offset (manual mode):',
            procedure.grooveAxialOffsetKw, 0))
        self.manualGrooveFields.append(AFXTextField(
            groove_matrix, 12, 'Patch angular offset, deg (manual mode):',
            procedure.grooveAngularOffsetKw, 0))

        # Tab 2B: attachment/cover parameters. This tab is hidden whenever
        # the groove operation is selected.
        self.attachmentTab = FXTabItem(
            self.tabs, '2  Cube / Sphere Cover')
        self.attachmentPage = FXVerticalFrame(
            self.tabs, FRAME_RAISED | LAYOUT_FILL_X | LAYOUT_FILL_Y)
        FXLabel(
            self.attachmentPage,
            'Use explicit rows/columns or calculate the largest complete '
            'grid from the picked face. Detached cover uses a positive gap '
            'and never touches or partitions the revolved body.',
            None, JUSTIFY_LEFT | LAYOUT_FILL_X)
        FXCheckButton(
            self.attachmentPage,
            'Automatically calculate rows/columns to fill the picked face',
            procedure.fitAttachmentsToFaceKw, 0)
        FXCheckButton(
            self.attachmentPage,
            'Detached cover: keep cubes/spheres close but not touching',
            procedure.detachedAttachmentsKw, 0)
        attachments = FXGroupBox(
            self.attachmentPage, 'Cube / sphere layout',
            FRAME_GROOVE | LAYOUT_FILL_X)
        attach_matrix = FXMatrix(
            attachments, 2, MATRIX_BY_COLUMNS | LAYOUT_FILL_X)
        self.attachRowsField = AFXTextField(
            attach_matrix, 12, 'Rows (manual mode):',
            procedure.attachmentRowsKw, 0)
        self.attachColumnsField = AFXTextField(
            attach_matrix, 12, 'Columns (manual mode):',
            procedure.attachmentColumnsKw, 0)
        AFXTextField(attach_matrix, 12,
                     'Cube normal thickness / sphere diameter:',
                     procedure.attachmentSizeKw, 0)
        self.cubeMeridianField = AFXTextField(
            attach_matrix, 12,
            'Cube meridian length (0 = use normal thickness):',
            procedure.attachmentMeridianSizeKw, 0)
        self.cubeCircumField = AFXTextField(
            attach_matrix, 12,
            'Cube circumferential width (0 = use normal thickness):',
            procedure.attachmentCircumSizeKw, 0)
        packing = AFXComboBox(
            attach_matrix, 34, 3, 'Ring / row arrangement:',
            procedure.packingKw, 0)
        packing.appendItem('Direct / aligned', 0)
        packing.appendItem('Alternating half-pitch stagger', 1)
        packing.appendItem('Progressive angular shift each ring', 2)
        self.rowShiftField = AFXTextField(
            attach_matrix, 12, 'Progressive shift per ring, deg:',
            procedure.attachmentRowShiftDegreesKw, 0)
        AFXTextField(attach_matrix, 12, 'Number of normal layers:',
                     procedure.attachmentLayersKw, 0)
        layer_packing = AFXComboBox(
            attach_matrix, 30, 2, 'Layer-to-layer arrangement:',
            procedure.attachmentLayerPackingKw, 0)
        layer_packing.appendItem('Direct stack', 0)
        layer_packing.appendItem('Stagger adjacent layers', 1)
        self.embedField = AFXTextField(
            attach_matrix, 12, 'Interface embed fraction:',
            procedure.embedFractionKw, 0)
        self.clearanceField = AFXTextField(
            attach_matrix, 12, 'Detached surface clearance:',
            procedure.attachmentClearanceKw, 0)

        # Tab 3: options shared by every operation.
        FXTabItem(self.tabs, '3  Mesh / Output')
        mesh_page = FXVerticalFrame(
            self.tabs, FRAME_RAISED | LAYOUT_FILL_X | LAYOUT_FILL_Y)
        FXLabel(
            mesh_page,
            'Sweep hexahedral is the default. If Abaqus cannot sweep the '
            'generated topology, the failed mesh is cleared and retried '
            'automatically with free C3D4.',
            None, JUSTIFY_LEFT | LAYOUT_FILL_X)
        meshing = FXGroupBox(
            mesh_page, 'Automatic partition and mesh',
            FRAME_GROOVE | LAYOUT_FILL_X)
        mesh_matrix = FXMatrix(
            meshing, 2, MATRIX_BY_COLUMNS | LAYOUT_FILL_X)
        AFXTextField(mesh_matrix, 14, 'Global mesh size:',
                     procedure.meshSizeKw, 0)
        mesh_type = AFXComboBox(
            mesh_matrix, 38, 4, 'Mesh / element type:',
            procedure.meshTypeKw, 0)
        mesh_type.appendItem('Sweep hex - C3D8R/C3D6/C3D4', 4)
        mesh_type.appendItem('Free tetrahedral - C3D4', 0)
        mesh_type.appendItem('Free quadratic tetrahedral - C3D10', 1)
        mesh_type.appendItem('Free modified tetrahedral - C3D10M', 2)
        library = AFXComboBox(
            mesh_matrix, 24, 2, 'Element library:',
            procedure.elementLibraryKw, 0)
        library.appendItem('Abaqus/Standard', 0)
        library.appendItem('Abaqus/Explicit', 1)
        FXCheckButton(
            meshing,
            'Create groove sweep layers (attachments are never partitioned)',
            procedure.autoPartitionKw, 0)
        FXCheckButton(
            mesh_page, 'Overwrite an existing same-name output part',
            procedure.overwriteKw, 0)

        self._syncOperationPages()

    def _syncOperationPages(self):
        operation = int(self.procedure.operationKw.getValue())
        fit_grooves = bool(
            self.procedure.fitGroovePatchToFaceKw.getValue())
        fit_attachments = bool(
            self.procedure.fitAttachmentsToFaceKw.getValue())
        detached = bool(self.procedure.detachedAttachmentsKw.getValue())
        groove_mode = int(
            self.procedure.grooveConstructionModeKw.getValue())
        packing = int(self.procedure.packingKw.getValue())
        state = (operation, fit_grooves, fit_attachments, detached,
                 groove_mode, packing)
        if state == self._visibleState:
            return
        self._visibleState = state
        if operation == 0:
            self.grooveTab.show()
            self.groovePage.show()
            self.attachmentTab.hide()
            self.attachmentPage.hide()
        else:
            self.grooveTab.hide()
            self.groovePage.hide()
            self.attachmentTab.show()
            self.attachmentPage.show()
        for field in self.manualGrooveFields:
            if fit_grooves:
                field.hide()
            else:
                field.show()
        if fit_attachments:
            self.attachRowsField.hide()
            self.attachColumnsField.hide()
        else:
            self.attachRowsField.show()
            self.attachColumnsField.show()
        if detached:
            self.embedField.hide()
            self.clearanceField.show()
        else:
            self.embedField.show()
            self.clearanceField.hide()
        if groove_mode == 1:
            self.grooveDepthField.hide()
            self.grooveRaisedThicknessField.show()
        else:
            self.grooveDepthField.show()
            self.grooveRaisedThicknessField.hide()
        if operation == 1:
            self.cubeMeridianField.show()
            self.cubeCircumField.show()
        else:
            self.cubeMeridianField.hide()
            self.cubeCircumField.hide()
        if packing == 2:
            self.rowShiftField.show()
        else:
            self.rowShiftField.hide()
        try:
            self.tabs.recalc()
            self.tabs.layout()
        except Exception:
            pass

    def processUpdates(self):
        """React immediately when the operation combo keyword changes."""
        self._syncOperationPages()
