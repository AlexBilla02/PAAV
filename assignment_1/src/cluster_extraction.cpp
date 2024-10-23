#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>

#include <thread>  
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"

#include <cmath>
#include <format>

#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;


// Funzione per calcolare la distanza euclidea
float calculateDistance(const pcl::PointXYZ& point, const pcl::PointXYZ& referencePoint)
{
    return std::sqrt(std::pow(point.x - referencePoint.x, 2) +
                     std::pow(point.y - referencePoint.y, 2) +
                     std::pow(point.z - referencePoint.z, 2));
}
//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
    my_visited_set_t visited;  // insieme di punti già visitati
    std::vector<pcl::PointIndices> clusters;  // vettore finale che conterrà i cluster

    // ciclo su ogni punto del cloud
    for (int i = 0; i < cloud->size(); ++i)
    {
        // Se il punto non è stato visitato allora continuo con l'analisi
        if (visited.find(i) == visited.end())
        {
            // Creo nuovo cluster
            std::vector<int> cluster;

            // Chiamo la funzione proximity per costruire il cluster
            proximity(cloud, i, tree, distanceTol, visited, cluster, setMaxClusterSize);

            // se la dimensione del cluster è inclusa nel range allora posso creare l'oggetto poind_indices dei cluster
            if (cluster.size() >= setMinClusterSize && cluster.size() <= setMaxClusterSize)
            {
                pcl::PointIndices point_indices;
                point_indices.indices = cluster;  // Aggiungo gli indici trovati nel cluster
                clusters.push_back(point_indices);  // Aggiungo il cluster al vettore finale
            }
        }
    }
    return clusters;  
}

void 
ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // TODO: 1) Downsample the dataset 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

   pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud (cloud);
   sor.setLeafSize (0.16f, 0.16f, 0.16f); //DOWNSAMPLE SET 
   sor.filter (*cloud_filtered); 
   std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;


    // 2) here we crop the points that are far away from us, in which we are not interested
    //non toccare, va bene così di default per il primo dataset
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
    cb.filter(*cloud_filtered); 
   
     
   

    // TODO: 3) Segmentation and apply RANSAC per andare a rimuovere il piano del pavimento/strada
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);    // Identifica il piano
    seg.setMethodType(pcl::SAC_RANSAC);       // Utilizza RANSAC per il fitting del modello
    seg.setMaxIterations(100);                // Numero massimo di iterazioni
    seg.setDistanceThreshold(0.30);            // Distanza massima di un punto per essere considerato inlier
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);  // Mantieni solo gli inliers (il piano)
    extract.filter(*cloud_plane);  // Estrazione del piano

    //togliere commento per visualizzare strada
    //renderer.RenderPointCloud(cloud_plane, "planeCloud", Color(0,1,0));

    extract.setNegative(true);   // Mantieni i punti che non sono parte del piano
    extract.filter(*cloud_filtered);  // Rimuovi il piano dalla nuvola di punti filtrata
    std::cerr << "PointCloud after RANSAC: " << cloud_filtered->width * cloud_filtered->height << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;




    // TODO: 5) Create the KDTree and the vector of PointIndices
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered); 
    // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
    std::vector<pcl::PointIndices> cluster_indices;
    #ifdef USE_PCL_LIBRARY
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

        //Set the spatial tolerance for new cluster candidates
        //If you take a very small value, it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster
        ec.setClusterTolerance (0.25); // 25cm

        ec.setMinClusterSize (100); //num minimo  di punti che deve avere il cluster
        ec.setMaxClusterSize (25000);   //num massimo di punti che può avere il cluster
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);
        std::cout<<cluster_indices.size()<<endl;
        
    #else
        // Optional assignment
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);
        cluster_indices = euclideanCluster(cloud_filtered, &treeM, 0.25, 100, 25000);
    #endif

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};
    //renderer.RenderPointCloud(cloud_filtered,"originalCloud",colors[2]);


    // Considero il punto centrale del dataset (centroide), che mi servirà per la distanza del veicolo dagli elementi circostanti
    pcl::PointXYZ centroid(0.0, 0.0, 0.0);
    

    int j = 0;
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    { 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud_filtered)[*pit]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
       

        //renderer.RenderPointCloud(cloud,"originalCloud"+std::to_string(clusterId),colors[2]);
        // TODO: 7) render the cluster and plane without rendering the original cloud 
        //<-- here
        //----------

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        // Calcolo del punto centrale del bounding box, che mi servirà per la distanza dal punto centrale (il mio veicolo)
        float boxCenterX = (minPt.x + maxPt.x) / 2;
        float boxCenterY = (minPt.y + maxPt.y) / 2;
        float boxCenterZ = (minPt.z + maxPt.z) / 2;
        //TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        Box box{minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};
           // Prendi il primo punto del cluster
        //pcl::PointXYZ firstPoint = cloud_cluster->points[0];

        //prendo il punto centrale del box per calcolare la distanza dal mio veicolo
        pcl::PointXYZ objectPoint(boxCenterX, boxCenterY, boxCenterZ);
        //Cambiare boxCenterX e boxCenterY in minPt.x e minPt.y per andare ad avere una distanza col punto più "vicino" del box e non un punto medio
        //ovvero in      pcl::PointXYZ objectPoint(minPt.x, minPt.y, boxCenterZ);



        // Calcola la distanza tra il centroide (0,0,0) e il punto del cluster
        float distanceFromCentroid = calculateDistance(objectPoint, centroid);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << distanceFromCentroid; //per settare la distanza con due valori decimali
        std::string mystring = ss.str();        
        // Converti la distanza in stringa da visualizzare
        std::string distanceText =  mystring + " m";

        // Posiziona il testo sopra il bounding box
        pcl::PointXYZ textPosition;
        textPosition.x = (minPt.x + maxPt.x) / 2;  // Coordinata x centrale del bounding box
        textPosition.y = (minPt.y + maxPt.y) / 2;  // Coordinata y centrale del bounding box
        textPosition.z = maxPt.z + 0.5;  // Posiziona leggermente sopra il bounding box

        // Renderizza il testo con il renderer
        renderer.addText(textPosition.x, textPosition.y, textPosition.z, distanceText);


        //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
        //please take a look at the function RenderBox to see how to color the box
        if(distanceFromCentroid<5 && (minPt.x>0 || maxPt.x>0))
                renderer.RenderBox(box, j,colors[0]);
        
        else
            renderer.RenderBox(box, j,colors[3]);
        ++clusterId;
        j++;
    }  

}


int main(int argc, char* argv[])
{
    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"../dataset_1"},
                                                boost::filesystem::directory_iterator{});
   
    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
        << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        

        renderer.SpinViewerOnce();
        //std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Ritardo di 1000 ms (1 secondo)
    }
}


