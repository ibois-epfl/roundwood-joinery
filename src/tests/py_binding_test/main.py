from compas.colors import Color
from compas.geometry import Pointcloud
from compas_viewer import Viewer

def main():
    
    viewer = Viewer()
    pointcloud = Pointcloud.from_bounds(10, 10, 10, 10)
    viewer.scene.add(pointcloud, pointcolor=Color(1.0,0.0,0.0), pointsize=10)

    viewer.show()


if __name__ == "__main__":
    main()
