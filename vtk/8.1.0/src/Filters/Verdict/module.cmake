vtk_module(vtkFiltersVerdict
  GROUPS
    StandAlone
  TEST_DEPENDS
    vtkIOLegacy
    vtkTestingCore
  KIT
    vtkFilters
  DEPENDS
    vtkCommonExecutionModel
    vtkverdict
  PRIVATE_DEPENDS
    vtkCommonCore
    vtkCommonDataModel
  )