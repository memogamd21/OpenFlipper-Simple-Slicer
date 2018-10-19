#ifndef SIMPLESLICERPLUGIN_HH
#define SIMPLESLICERPLUGIN_HH

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/BackupInterface.hh>
#include <OpenFlipper/BasePlugin/ScriptInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>
#include <OpenFlipper/BasePlugin/RPCInterface.hh>
#include <OpenFlipper/common/Types.hh>

#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/PolyLine/PolyLine.hh>
#include <ObjectTypes/PolyLineCollection/PolyLineCollection.hh>

#include <QObject>
#include <QMenuBar>
#include <list>


#include "SimpleSlicerToolbar.hh"

struct lineSegment{
    bool valid;
    ACG::Vec3d start;
    ACG::Vec3d end;
    lineSegment(ACG::Vec3d startP,ACG::Vec3d endP): valid(false),start(startP),end(endP){}
    lineSegment(){start = ACG::Vec3d(0,0,0); end = start; valid = false; }
};

struct faceVertices{
  ACG::Vec3d a;
  ACG::Vec3d b;
  ACG::Vec3d c;
  faceVertices(ACG::Vec3d ap, ACG::Vec3d bp, ACG::Vec3d cp):a(ap),b(bp),c(cp){}
};
class SimpleSlicerPlugin : public QObject, BaseInterface , ToolboxInterface, BackupInterface, LoggingInterface, ScriptInterface, RPCInterface, LoadSaveInterface
{
	Q_OBJECT
	Q_INTERFACES(BaseInterface)
	Q_INTERFACES(ToolboxInterface)
	Q_INTERFACES(BackupInterface)
	Q_INTERFACES(LoggingInterface)
	Q_INTERFACES(ScriptInterface)
	Q_INTERFACES(LoadSaveInterface)
	Q_INTERFACES(RPCInterface)

#if QT_VERSION >= 0x050000
	Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-SimpleSlicer")
#endif

	signals:
	//BaseInterface
	void updateView();

	//LoggingInterface
	void log(Logtype _type, QString _message);
	void log(QString _message);

	// ToolboxInterface
	void addToolbox(QString _name, QWidget* _toolbox, QIcon* icon);

	void addEmptyObject( DataType _type, int& _id);
    void updatedObject(int, const UpdateType& _type);
public:

	// BaseInterface
	QString name() { return (QString("Simple Slicer Plugin")); };
	QString description( ) { return (QString("Simple Slicer plugin")); };
    QString version() { return QString("1.0"); };

	private slots:

	///Plugin Initialization
	void initializePlugin();

    ///Generate the baseplate
	int generateGroundPlate();

	///Focus onto the target object
	void zoomToObject();

	///Get id of the targeted object
	int getTargetId();

	///Translate object to the center of the baseplate
	void translateObject();

	///Scale object according to the user defined scale factor
	void scaleObject();



	//Sample the overhang structures


	public slots:

    void slice();
    PolyLine* sliceLayer(int layerNumber, double layerHeight);
    lineSegment intersectTriangle(std::vector<ACG::Vec3d> points, int layerNumber, double layerHeight);
	private:


	SimpleSlicerToolbarWidget* tool_;
	QIcon* toolIcon_;
    PolyLineCollectionObject* pCO;

};

#endif //SIMPLESLICERPLUGIN_HH
