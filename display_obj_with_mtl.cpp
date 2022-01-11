//
// Created by Harold on 2022/1/11.
//

// PCL not show multiple textures... so need to use VTK directly
// https://github.com/PointCloudLibrary/pcl/issues/2252
// https://github.com/PointCloudLibrary/pcl/issues/2095
// pcl::visualization::PCLVisualizer vis("mesh");
// vis.addTextureMesh(mesh, "texture-mesh");
// vis.setSize(2000, 1000);
// vis.spin();

#pragma warning(disable : 4996)
#pragma warning(disable : 4819)

#include <boost/filesystem.hpp>

#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkOBJImporter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTexture.h>

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cout
            << "Usage: " << argv[0]
            << " objfile mtlfile texturepath e.g. xx.obj"
            << std::endl;
        return EXIT_FAILURE;
    }

    std::string objpath;
    if (boost::filesystem::extension(argv[1]) == ".obj")
        objpath = argv[1];
    else
    {
        std::cerr << "only support .obj file with .mtl" << std::endl;
        return EXIT_FAILURE;
    }
    std::string mtlpath = boost::filesystem::path(objpath).replace_extension(".mtl").string();

    vtkNew<vtkOBJImporter> importer;
    importer->SetFileName(objpath.c_str());
    importer->SetFileNameMTL(mtlpath.c_str());  // assume .mtl is within the same folder with .obj
    importer->SetTexturePath("");  // assume texture filepaths in .mtl are absolute paths

    vtkNew<vtkNamedColors> colors;

    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkRenderWindow> renWin;
    vtkNew<vtkRenderWindowInteractor> iren;

    renderer->SetBackground2(colors->GetColor3d("Silver").GetData());
    renderer->SetBackground(colors->GetColor3d("Gold").GetData());
    renderer->GradientBackgroundOn();
    renWin->AddRenderer(renderer);
    renderer->UseHiddenLineRemovalOn();
    renWin->AddRenderer(renderer);
    renWin->SetSize(640, 480);
    renWin->SetWindowName("OBJImporter");

    iren->SetRenderWindow(renWin);
    importer->SetRenderWindow(renWin);
    importer->Update();

    auto actors = renderer->GetActors();
    actors->InitTraversal();
    // std::cout << "There are " << actors->GetNumberOfItems() << " actors" << std::endl;

    for (vtkIdType a = 0; a < actors->GetNumberOfItems(); ++a)
    {
        //std::cout << importer->GetOutputDescription(a) << std::endl;

        vtkActor *actor = actors->GetNextActor();

        // OBJImporter turns texture interpolation off
        if (actor->GetTexture())
        {
            //std::cout << "Has texture\n";
            actor->GetTexture()->InterpolateOn();
        }

        vtkPolyData *pd =
            dynamic_cast<vtkPolyData *>(actor->GetMapper()->GetInput());

        vtkPolyDataMapper *mapper =
            dynamic_cast<vtkPolyDataMapper *>(actor->GetMapper());
        mapper->SetInputData(pd);
    }
    renWin->Render();
    iren->Start();

    return EXIT_SUCCESS;
}
