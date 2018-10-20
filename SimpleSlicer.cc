
#include "SimpleSlicer.hh"
#include <ObjectTypes/Plane/Plane.hh>
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>
#include <OpenFlipper/BasePlugin/PluginFunctionsViewControls.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>
#include <ACG/Utils/VSToolsT.hh>
#include "OpenFlipper/INIFile/INIFile.hh"
#include <ObjectTypes/PolyMesh/PolyMesh.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ACG/Geometry/bsp/TriangleBSPT.hh>
#include "OpenFlipper/BasePlugin/PluginFunctions.hh"
#include <ObjectTypes/PolyLineCollection/PolyLineCollection.hh>

#include <QTime>
#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <list> 		// std::list
#include <utility>      // std::pair
#include <queue>		// std::queue

#include <omp.h>
#include <ACG/GL/gl.hh>
#if QT_VERSION >= 0x050000
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <QFile>
#include <QTextStream>

#define EPS (1e-6)
#define GC_MOVE 2
#define GC_WALL_INNER 3
#define GC_WALL_OUTER 0
#define GC_INFILL 1
#define GC_TOPBOTTOM 4






void SimpleSlicerPlugin::initializePlugin()
{
    QIcon* icon = new QIcon(OpenFlipper::Options::iconDirStr()+OpenFlipper::Options::dirSeparator()+"simpleSlicer.png");
    tool_ = new SimpleSlicerToolbarWidget();
    QSize size(300, 300);
    tool_->resize(size);
    connect(tool_->GenerateGroundPlateButton, SIGNAL(clicked()), this, SLOT(generateGroundPlate()));
    connect(tool_->SliceButton, SIGNAL(clicked()), this, SLOT(slice()));
    connect(tool_->TranslateObjectButton, SIGNAL(clicked()), this, SLOT(translateObject()));
    connect(tool_->ScaleButton, SIGNAL(clicked()), this, SLOT(scaleObject()));
    emit addToolbox("SimpleSlicer", tool_, icon);
    int pid = -1;
    emit addEmptyObject(DATA_POLY_LINE_COLLECTION,pid);
    PluginFunctions::getObject(pid,pCO);
    pCO->target(false);

}


int SimpleSlicerPlugin::generateGroundPlate()
{
    int objectId = -1;
    emit addEmptyObject( DATA_PLANE, objectId );
    PlaneObject* object;
    if ( !PluginFunctions::getObject(objectId,object) ) {
        emit log(LOGERR,"Unable to create new Object");
        return -1;
    }
    PlaneNode* planeNode = object->planeNode();
    Plane p = planeNode->getPlane();

    //PluginFunctions::plane
    p.setPlane(ACG::Vec3d(111.5,111.5,0),ACG::Vec3d(0,0,1));
    p.setSize(223,223);
    planeNode->setPlane(p);
    planeNode->update();
    return 0;
}
void SimpleSlicerPlugin::translateObject()
{
    int targetId = getTargetId();
    TriMeshObject* target = PluginFunctions::triMeshObject(targetId);
    ACG::Vec3d bbmin;
    ACG::Vec3d bbmax;
    target->getBoundingBox(bbmin, bbmax);
    RPC::callFunction("move","translate",targetId, ACG::Vec3d(111.5,111.5,0)-ACG::Vec3d(bbmin[0]+(bbmax[0]-bbmin[0])/2,bbmin[1]+(bbmax[1]-bbmin[1])/2,bbmin[2]) );
}

void SimpleSlicerPlugin::scaleObject()
{
    int targetId = getTargetId();
    TriMeshObject* target = PluginFunctions::triMeshObject(targetId);
    ACG::Vec3d bbmin;
    ACG::Vec3d bbmax;
    target->getBoundingBox(bbmin, bbmax);
    //double md[16] = {205*tool_->ScaleFactor->value()/111.5/(bbmin-bbmax).length(),0,0,0,0,205*tool_->ScaleFactor->value()/111.5/(bbmin-bbmax).length(),0,0,0,0,205*tool_->ScaleFactor->value()/111.5/(bbmin-bbmax).length(),0 };
    double md[16] = {205*tool_->ScaleFactor->value()/100.0/(bbmin-bbmax).length(),0,0,0,0,205*tool_->ScaleFactor->value()/100.0/(bbmin-bbmax).length(),0,0,0,0,205*tool_->ScaleFactor->value()/100.0/(bbmin-bbmax).length(),0 };
    ACG::Matrix4x4d m (md);
    RPC::callFunction("move","transform",targetId, m);
    translateObject();
    zoomToObject();
}

void SimpleSlicerPlugin::zoomToObject()
{
    int targetId = getTargetId();
    TriMeshObject* target = PluginFunctions::triMeshObject(targetId);
    ACG::Vec3d bbmin;
    ACG::Vec3d bbmax;
    target->getBoundingBox(bbmin, bbmax);

    ACG::Vec3d bbcenter = (bbmax + bbmin) * 0.5;

    double bbradius = (bbmax - bbmin).norm();

    ACG::Vec3d eye = bbcenter + (PluginFunctions::eyePos() - bbcenter).normalize() * bbradius ;

    PluginFunctions::flyTo(eye, bbcenter );
}

int SimpleSlicerPlugin::getTargetId()
{
    // Get source and target objects
    BaseObjectData* targetObject = 0;
    for ( PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS) ; o_it != PluginFunctions::objectsEnd(); ++o_it) {

        if ( o_it->dataType(DATA_TRIANGLE_MESH)  || o_it->dataType(DATA_POLY_MESH) ) {

            // If we found a second target, something is wrong!
            if ( targetObject != 0 ) {
                emit log(LOGERR,tr("Please select one source and one target mesh to compare! Source will be the reference mesh."));
                return -1;
            }

            targetObject = (*o_it);
            break;
        }
    }

    return targetObject->id();
}

void SimpleSlicerPlugin::slice()
{
    //Find current target and get bounding box
    int targetId = getTargetId();
    TriMeshObject* target = PluginFunctions::triMeshObject(targetId);
    ACG::Vec3d bbmin;
    ACG::Vec3d bbmax;
    target->getBoundingBox(bbmin, bbmax);
    auto layerHeight = tool_->sb_layerHeight->value();

     auto objectHeight = bbmax[2]-bbmin[2];

    /* Calculate number of layers and
    * loop over all layers, and compute each slice using:
    *
    * Polyline* pl = sliceLayer(layerNumber, layerHeight);
    * if(!pl) continue;
    *
    * and add the line into the collection of lines:
    *
    * pCO->collection()->add_poly_line(pl);
    *
    */
    //#######START###########
    auto numofLayers = std::ceil(objectHeight/layerHeight);
    for (int i=0 ; i<numofLayers ; i++){
        PolyLine* pl = sliceLayer(i, layerHeight);
        if(!pl)continue;

        pCO->collection()->add_poly_line(pl);
    }

    //########END##########

    emit updatedObject(pCO->id(),UPDATE_ALL);

}

PolyLine* SimpleSlicerPlugin::sliceLayer(int layerNumber, double layerHeight)
{
    int targetId = getTargetId();
    TriMesh& mesh = [&]() -> TriMesh& {
            TriMesh* pmesh;
            PluginFunctions::getMesh(targetId, pmesh);
            return *pmesh;
}();
    PolyLine& outwall = *(new PolyLine());
    std::vector<lineSegment> lineSoup;


    /*
     * Loop over all Triangles of the mesh and find the ones intersecting the current layer.
     * Hint: Store all points of the triangle in a vector:
     * std::vector<ACG::Vec3d> points;
     *
     * Use the function
     * lineSegment l = intersectTriangle( points,layerNumber,layerHeight);
     * to obtain the current line segment. Check if it is valid and push it into the lineSoup.
     */
    //########START##########

    for (auto face : mesh.faces()){
std::vector<ACG::Vec3d> points;
         for ( auto singlevertex : mesh.fv_range(face)){
             ACG::Vec3d point = mesh.point(singlevertex);
             points.push_back(point);
         }

     lineSegment l = intersectTriangle( points,layerNumber,layerHeight);
     if (l.valid){
       linesoup.push_back(l); //this pushes this line into the linesoupdatastructure
     }
    }
    //########END###########

    if(lineSoup.size() == 0)return nullptr;


    outwall.add_point(lineSoup[0].start);
    outwall.add_point(lineSoup[0].end);
    lineSoup[0].valid = false;

    while(true)
    {
        /*
     * Check if the current Last Point of the outwall is somewhere else in the linesoup by looping
     * over the linesoup and check position difference between points. The point difference can
     * be computed using (p0-p1).length(); Points are close when their difference is < 0.00001.
     * You can get the last point of the outwall by
     * ACG::Vec3d last = outwall.point(outwall.n_vertices()-1);
     *
     * Note: Start and End Points might be backwards, so you have to check whether
     * the current point is equal to start or end point of the other lines.
     *
     * Hint: Mark all already used points by set the .valid property to false;
     * Once you cannot find any other point, exit the loop using the
     * break;
     * statement
    }
    emit updatedObject(targetId,UPDATE_ALL);
    return &outwall;
}

lineSegment SimpleSlicerPlugin::intersectTriangle(std::vector<ACG::Vec3d> points, int layerNumber, double layerHeight)
{
    lineSegment line;
    std::vector<ACG::Vec3d> intersections;

    /*
     * Calculate the current height of the slice you are processing
     */
    //########START##########
    auto currentHeight = layerNumber * layerHeight ;
    //########END###########


    /*
     * Check if the line is actually intersecting the plane.
     * If you find that it is not intersecting, set:
     * line.valid = false;
     * return line;
     * to dismiss the current triangle.
     */
    //########START##########
    std::vector<int> is_above(3); // 0: below    1: above
    for(int i=0;i<3;i++){
        if(points[i][2]<currentHeight){
            is_above[i]=0;
        }else{is_above[i]=1;
        }
    }
    if(is_above[1]==is_above[2]&& is_above[1]==is_above[0]){
     line.valid = false;
     return line;
    }

    //########END###########

    /*
     * If the triangle is intersecting, you now have to identify which edge intersects the plane.
     * For each edge of the triangle, check if you can find the intersection point. If you can find it, push
     * it into the intersections vector by using:
     * intersections.push_back( Some 3D point);
     */
    //########START##########

     for (int i=0;i<3; i++){
     if(is_above[i]!=is_above[(i+1)%3]){
     auto a = points[i];
     auto b =points[(i+1)%3];
     auto zdist = abs(b[2]-a[2]);
     auto intersectdist = abs(currentHeight - a[2]);
     auto ratio = intersectdist / zdist;
     auto intersectionpoint = a + ratio * (b-a);
     intersections.push_back(intersectionpoint);
     }
     }
         // this finds the ratio of the intersecting plane with the whole z distance

    //########END###########

    /*
     * Check if the number of intersections is correct.
     * If it is correct, fill
     * line.start = some3Dpoint;
     * line.end = some3Dpoint;
     * and set it's valid to true.
     */

    if(intersections.size()<2)line.valid= false;
    else
    {
        line.start = intersections[0];
        line.end = intersections[1];
        line.valid = true;
    }

    return line;
}
