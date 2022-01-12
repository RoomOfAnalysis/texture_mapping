//
// Created by Harold on 2022/1/12.
//

#pragma warning(disable : 4996)
#pragma warning(disable : 4819)

#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkOBJImporter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTexture.h>

#include <vtkSmartPointer.h>
#include <vtkObjectFactory.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <vtkSmartPointer.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkProperty.h>

#include <sstream>
#include <boost/filesystem.hpp>

// handle key press events
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static KeyPressInteractorStyle *New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

    virtual void OnKeyPress() override
    {
        vtkRenderWindowInteractor *rwi = this->Interactor;
        std::string key = rwi->GetKeySym();
        // remove one texture and corresponding camera pose lines
        if (key == "Up")
        {
            if (cursor > 0)
            {
                this->GetCurrentRenderer()->RemoveActor(texture_and_cam_pairs[cursor].first);
                this->GetCurrentRenderer()->RemoveActor(texture_and_cam_pairs[cursor].second);
                cursor--;
            }
        }
        // add one texture and corresponding camera pose lines
        if (key == "Down")
        {
            if (cursor >= 0 && cursor < int(texture_and_cam_pairs.size()) - 1)
            {
                cursor++;
                this->GetCurrentRenderer()->AddActor(texture_and_cam_pairs[cursor].first);
                this->GetCurrentRenderer()->AddActor(texture_and_cam_pairs[cursor].second);
            }
        }
        vtkInteractorStyleTrackballCamera::OnKeyPress();
        this->GetCurrentRenderer()->GetRenderWindow()->Render();
    }

    void SetTextureAndCamPairs(std::vector<std::pair<vtkSmartPointer<vtkActor>, vtkSmartPointer<vtkActor>>> texture_and_cam_pairs_)
    {
        texture_and_cam_pairs = texture_and_cam_pairs_;
    }
private:
    std::vector<std::pair<vtkSmartPointer<vtkActor>, vtkSmartPointer<vtkActor>>> texture_and_cam_pairs;
    int cursor = 0;
};
vtkStandardNewMacro(KeyPressInteractorStyle);

// axis
vtkSmartPointer<vtkOrientationMarkerWidget> SetUpAxesWidget(vtkRenderWindowInteractor* interactor);

// draw 5-points camera
vtkSmartPointer<vtkActor> DrawCamera(std::string const& camera_file_path);

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
    vtkNew<KeyPressInteractorStyle> style;
    iren->SetInteractorStyle(style);
    style->SetCurrentRenderer(renderer);

    renderer->SetBackground2(colors->GetColor3d("Silver").GetData());
    renderer->SetBackground(colors->GetColor3d("Gold").GetData());
    renderer->GradientBackgroundOn();
    renWin->AddRenderer(renderer);
    renderer->UseHiddenLineRemovalOn();
    renWin->AddRenderer(renderer);
    renWin->SetSize(2000, 1000);
    renWin->SetWindowName("DisplayOBJ");

    iren->SetRenderWindow(renWin);
    importer->SetRenderWindow(renWin);
    importer->Update();

    // textures
    auto actors = renderer->GetActors();
    actors->InitTraversal();
    auto textures_number = actors->GetNumberOfItems();
    // std::cout << "There are " << textures_number << " textures" << std::endl;

    std::vector<std::pair<vtkSmartPointer<vtkActor>, vtkSmartPointer<vtkActor>>> texture_and_cam_pairs;
    texture_and_cam_pairs.reserve(textures_number - 1);
    for (vtkIdType a = 0; a < textures_number; ++a)
    {
        vtkSmartPointer<vtkActor> actor = actors->GetNextActor();
        // if (actor->GetTexture())
        // {
        //   actor->GetTexture()->InterpolateOn();  // turns texture interpolation on
        // }
        // vtkPolyData *pd = dynamic_cast<vtkPolyData *>(actor->GetMapper()->GetInput());
        // vtkPolyDataMapper *mapper = dynamic_cast<vtkPolyDataMapper *>(actor->GetMapper());
        // mapper->SetInputData(pd);

        // camera
        if (a < textures_number - 1)
        {
            auto info_str = importer->GetOutputDescription(a);
            std::string const pattern1 = "file ";
            std::string const pattern2 = " diffuse";
            auto start = info_str.find(pattern1) + pattern1.size();
            auto end = info_str.find(pattern2, start);
            auto texture_path = info_str.substr(start, end - start);
            //std::cout << texture_path << std::endl;
            auto cam_file_path = boost::filesystem::path(texture_path).replace_extension(".cam").string();
            texture_and_cam_pairs.emplace_back(actor, DrawCamera(cam_file_path));

            //std::cout << "removed: " << importer->GetOutputDescription(a) << std::endl;
            renderer->RemoveActor(actor);
        }
    }
    style->SetTextureAndCamPairs(texture_and_cam_pairs);

    // axes
    auto widget = SetUpAxesWidget(iren);
    widget->SetEnabled(1);
    widget->InteractiveOn();

    renWin->Render();
    iren->Start();

    return EXIT_SUCCESS;
}

vtkSmartPointer<vtkOrientationMarkerWidget> SetUpAxesWidget(vtkRenderWindowInteractor* interactor)
{
    vtkNew<vtkNamedColors> colors;
    // axes
    vtkNew<vtkAxesActor> axes;
    axes->SetCylinderRadius(0.03);
    axes->SetShaftTypeToCylinder();
    axes->SetTotalLength(1.5, 1.5, 1.5);

    // widget
    vtkNew<vtkOrientationMarkerWidget> widget;
    double rgba[4]{0.0, 0.0, 0.0, 0.0};
    colors->GetColor("Carrot", rgba);
    widget->SetOutlineColor(rgba[0], rgba[1], rgba[2]);
    widget->SetOrientationMarker(axes);
    widget->SetInteractor(interactor);
    widget->SetViewport(0.0, 0.0, 0.4, 0.4);
    return widget;
}

// 8 lines
vtkSmartPointer<vtkActor> DrawCamera(std::string const& camera_file_path)
{
    vtkNew<vtkActor> actor;
    std::ifstream ifs;
    ifs.open(camera_file_path.c_str(), std::ios::in);
    if (!ifs.is_open())
    {
        std::cerr << "Error opening file: " << camera_file_path << std::endl;
        return actor;
    }

    vtkNew<vtkPolyData> linesPolyData;
    vtkNew<vtkPoints> pts;
    std::string line;
    char c;
    double x = 0, y = 0, z = 0;
    // line 1
    while (std::getline(ifs, line))
    {
        std::stringstream ss(line);
        ss >> c >> x >> c >> y >> c >> z;
        pts->InsertNextPoint(x, y, z);
        ss >> c >> c >> x >> c >> y >> c >> z;
        pts->InsertNextPoint(x, y, z);
    }
    linesPolyData->SetPoints(pts);

    vtkNew<vtkLine> line1, line2, line3, line4, line5, line6, line7, line8;
    line1->GetPointIds()->SetId(0, 0);
    line1->GetPointIds()->SetId(1, 1);
    line2->GetPointIds()->SetId(0, 2);
    line2->GetPointIds()->SetId(1, 3);
    line3->GetPointIds()->SetId(0, 4);
    line3->GetPointIds()->SetId(1, 5);
    line4->GetPointIds()->SetId(0, 6);
    line4->GetPointIds()->SetId(1, 7);
    line5->GetPointIds()->SetId(0, 8);
    line5->GetPointIds()->SetId(1, 9);
    line6->GetPointIds()->SetId(0, 10);
    line6->GetPointIds()->SetId(1, 11);
    line7->GetPointIds()->SetId(0, 12);
    line7->GetPointIds()->SetId(1, 13);
    line8->GetPointIds()->SetId(0, 14);
    line8->GetPointIds()->SetId(1, 15);

    vtkNew<vtkCellArray> lines;
    lines->InsertNextCell(line1);
    lines->InsertNextCell(line2);
    lines->InsertNextCell(line3);
    lines->InsertNextCell(line4);
    lines->InsertNextCell(line5);
    lines->InsertNextCell(line6);
    lines->InsertNextCell(line7);
    lines->InsertNextCell(line8);
    linesPolyData->SetLines(lines);

    vtkNew<vtkNamedColors> namedColors;
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
    colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
    colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
    colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
    colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
    colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
    colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
    colors->InsertNextTypedTuple(namedColors->GetColor3ub("Tomato").GetData());
    linesPolyData->GetCellData()->SetScalars(colors);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(linesPolyData);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(4);
    actor->GetProperty()->SetOpacity(0.5);
    return actor;
}