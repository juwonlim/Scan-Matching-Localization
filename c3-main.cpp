//hazemfahmyy의 코드를 참조함
#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>

// pcl 라이브러리 및 필요한 헤더 포함
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;

// 키보드 이벤트 처리 함수
// 키보드 입력에 따라 차량의 조작 상태를 변경
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0)); // 오른쪽 회전
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); // 왼쪽 회전
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0)); // 전진
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); // 후진
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true; // 뷰를 갱신하도록 설정
	}
}

// 차량 조작 상태를 설정
// 입력된 ControlState를 기반으로 차량의 스로틀, 핸들, 브레이크를 설정
void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){  //전진
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){ // 후진
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f); //스티어링 조절
	state.brake = response.b; //브레이크 설정
}

// 차량의 포즈를 시각화
// 현재 포즈를 기반으로 차량의 위치와 회전 상태를 3D 뷰어에 표시
void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}


// NDT 함수 정의
// NDT: 라이다 데이터와 맵 데이터를 정렬하여 최적의 변환 행렬을 반환
Eigen::Matrix4d NDT(PointCloudT::Ptr mapCloud, PointCloudT::Ptr source, Pose startingPose, int iterations, int resolution) 
{   
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // NDT 파라미터 설정(NDT 매개변수 설정)
    ndt.setTransformationEpsilon(1e-4);  // 변환 종료 조건
    ndt.setResolution(resolution); //resolution, // 셀 크기
    // 입력 데이터 설정  
    ndt.setInputTarget(mapCloud);    // 맵 데이터를 타겟으로 설정
    pcl::console::TicToc time;
    time.tic ();
    // 초기 추정값 계산
    Eigen::Matrix4f init_guess = transform3D(
        startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, 
        startingPose.position.x, startingPose.position.y, startingPose.position.z).cast<float>();

     // 반복 횟수 설정
    ndt.setMaximumIterations(iterations); // 최대 반복 횟수 설정
    ndt.setInputSource(source); // 라이다 데이터를 소스로 설정

    // NDT 실행 및 변환된 점 구름 저장
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*cloud_ndt, init_guess);

    // 결과 확인 및 변환 행렬 반환 
    cout << "NDT converged?: " << ndt.hasConverged() << " Score: " << ndt.getFitnessScore() <<  endl;
    Eigen::Matrix4d transformed = ndt.getFinalTransformation ().cast<double>();
    return transformed;
}

int main(){
    
    //CARLA 클라이언트 설정
	auto client = cc::Client("localhost", 2000); //CARLA클라이언트 초기화
	client.SetTimeout(2s);
	auto world = client.GetWorld();
    
    // 차량과 라이다 센서 설정
	auto blueprint_library = world.GetBlueprintLibrary(); // 차량과 센서 블루프린트 가져오기
	auto vehicles = blueprint_library->Filter("vehicle");
	auto map = world.GetMap();
	
    //차량 생성 및 초기화
    auto transform = map->GetRecommendedSpawnPoints()[1]; // 차량 스폰 위치 설정
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);  // 차량 생성

	//Create lidar
    //라이다 센서 설정
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions
    
    // 라이다 범위 설정
    // CANDO: Can modify lidar values to get different scan resolutions
    lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "60");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0)); //초기포즈 정의

	// Load map
    // 맵 데이터 로드
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

    
    
    
    //NDT설정
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);

	lidar->Listen([&new_scan, &lastScanTime, &scanCloud](auto data){ //라이다 데이터 수신하는 코드

		if(new_scan){
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
			for (auto detection : *scan){  //기존 detection.point.x에서 point단어 삭제 --(시뮬레이터 코드 업데이트 되었기에)
				if((detection.x*detection.x + detection.y*detection.y + detection.z*detection.z) > 8.0){ // Don't include points touching ego ->>>?무슨뜻
                                                                                                         // 조건: 차량 근처(반지름 8m 이내)의 포인트 제거
					pclCloud.points.push_back(PointT(-detection.y, detection.x, detection.z));            // 차량과의 거리를 기준으로 데이터를 필터링하여 노이즈를 줄임, 멘토님이 이렇게 -detection.y로 하도록 정보제공
				}
			}
			//라이다 데이터 수집 및 처리 완료 상태를 확인하고, 새 데이터를 수신하도록 플래그를 제어하는 역할
			if(pclCloud.points.size() > 5000){ // CANDO: Can modify this value to get different scan resolutions 
			                //pclCloud.points.size()는 현재 라이다 데이터로 수집된 포인트의 수를 확인,포인트가 5000개 이상일 경우 데이터를 처리할 준비가 완료되었다고 간주
							//이 숫자(5000)는 해상도(스캔 밀도)에 영향을 미치며, 조정 가능하도록 코드에 CANDO로 주석을 추가한 것
				lastScanTime = std::chrono::system_clock::now(); //lastScanTime에 현재 시간을 저장
				*scanCloud = pclCloud; // 필터링된(현재 수집된) 포인트 클라우드를 scanCloud에 복사,scanCloud는 이후 프로세싱(예: NDT 정렬)에 사용될 데이터
				new_scan = false;  // 데이터 수집 완료 플래그 설정 , new_scan = false로 설정하여, 새 데이터를 추가적으로 수신하지 않도록 만듬
				                   //이는 데이터가 처리 완료될 때까지 불필요한 데이터 수신을 방지
			}
		}
	});
	
	Pose poseRef(Point(vehicle->GetTransform().location.x, 
	            vehicle->GetTransform().location.y, 
				vehicle->GetTransform().location.z), 
				Rotate(vehicle->GetTransform().rotation.yaw * pi/180, 
				vehicle->GetTransform().rotation.pitch * pi/180, 
				vehicle->GetTransform().rotation.roll * pi/180));
	double maxError = 0;

	while (!viewer->wasStopped())
  	{
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s); // 1초마다 시뮬레이터 상태를 갱신
		}
		if(refresh_view){
            // 카메라 위치 갱신
			viewer->setCameraPosition(
                pose.position.x, pose.position.y, 60, 
                pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");

        // 초기 포즈 설정
        //Pose pose(Point(0, 0, 0), Rotate(0, 0, 0)); // 초기 포즈 정의 (기본값 설정) , while문 내에 두면 안될듯해서 주석처리함
		// 차량의 현재 위치와 회전을 기준 좌표계(poseRef)에 대해 상대적으로 계산
		// 실시간 갱신을 위해 pose truePose는 while문 내부에 위치
		// 랜더링 동기화 : truePose는 drawCar 함수에서 차량의 현재 상태를 렌더링하는 데 사용. 따라서 루프마다 최신 정보를 반영해야 함
        Pose truePose = Pose(
                        Point(
                            vehicle->GetTransform().location.x, 
                            vehicle->GetTransform().location.y, 
                            vehicle->GetTransform().location.z
                            ), 
                        Rotate(
                            vehicle->GetTransform().rotation.yaw * pi/180,  // 차량의 현재 yaw 값을 라디안으로 변환
                            vehicle->GetTransform().rotation.pitch * pi/180,  // 차량의 현재 pitch 값을 라디안으로 변환
                            vehicle->GetTransform().rotation.roll * pi/180
                            )
                        ) - poseRef;  // 차량의 현재 roll 값을 라디안으로 변환 ,// 차량의 초기 위치와 회전을 기준으로 참조 포즈(poseRef)를 설정
		
        drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);  // 차량의 현재 포즈를 시각화
		double theta = truePose.rotation.yaw; // 현재 yaw 값
		double stheta = control.steer * pi/4 + theta; // 스티어링 보정 계산
		
        // 스티어링 방향을 렌더링
        // truePose: 차량의 현재 포즈
        // theta: 현재 회전 상태
        // stheta: 스티어링 보정 상태
        viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), 
                                truePose.position.y+2*sin(theta),truePose.position.z),  
                                Point(truePose.position.x+4*cos(stheta), 
                                truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));

        // 차량 제어 상태 업데이트
		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control); // 차량 제어 함수 호출
			vehicle->ApplyControl(control);  // 차량 상태 적용
         	cout << "applied control" << endl;
		}

  		viewer->spinOnce ();// 뷰어 갱신
		
		if(!new_scan){
			cout << "begin scan" << endl; //endl뒤에 ; 이게 누락되서 에러났었음
			new_scan = true;
			           
            // TODO: (Filter scan using voxel filter)
            // TODO: (복셀 필터를 사용하여 스캔 데이터 필터링)
            // 입력된 라이다 데이터(scanCloud)에서 불필요한 데이터를 제거하고
            // 간소화된 데이터(cloudFiltered)를 생성
			//라이다 데이터 필터링
            pcl::VoxelGrid<PointT> vg; //declare voxelgrid
			vg.setInputCloud(scanCloud); // 스캔 데이터 입력
			double filterRes = 1.0; //resoultion
			vg.setLeafSize(filterRes, filterRes, filterRes); // leaf size
			vg.filter(*cloudFiltered); //  필터링된 데이터를 cloudFiltered에 저장
			// TODO: Find pose transform by using ICP or NDT matching
			
            //Eigen::Matrix4d transform = ICP(mapCloud, cloudFiltered, pose, 100); //ICP사용안함으로써 주석처리
			Eigen::Matrix4d transform = NDT(mapCloud, cloudFiltered, pose, 130, 10); //NDT함수사용
          	pose = getPose(transform);  // 변환 행렬을 기반으로 현재 포즈 업데이트
			
            // 변환된 스캔 데이터를 생성 및 렌더링
			// 매칭된 변환 행렬을 사용해 라이다 데이터를 맵에 맞게 변환해야 함
            // TODO: Transform scan so it aligns with ego's actual pose and render that scan
			PointCloudT::Ptr ScanCorrected (new PointCloudT);// 변환된 스캔 데이터를 저장할 변수
			pcl::transformPointCloud(*cloudFiltered, *ScanCorrected, transform);// 변환된 스캔 데이터 생성
            viewer->removePointCloud("scan"); // 기존의 스캔 데이터 제거
            // TODO: Change `scanCloud` below to your transformed scan
			renderPointCloud(viewer, ScanCorrected, "scan", Color(1,0,0) ); // 변환된 데이터를 렌더링
			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
          

          //여기서부터
          // 에러 및 이동 거리 계산
          // poseError: 현재 포즈와 참조 포즈 간의 거리
          // maxError: 현재까지의 최대 포즈 에러
          	double poseError = sqrt( (truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + (truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y) );
			if(poseError > maxError)
				maxError = poseError;
			double distDriven = sqrt( (truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y) );
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			if(maxError > 1.2 || distDriven >= 170.0 ){
				viewer->removeShape("eval");
			if(maxError > 1.2){
				viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
			}
			else{
				viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
			}
		}

			pclCloud.points.clear(); // 포인트 클라우드 초기화
		}
  	}
	return 0; //여기까지 유다시티 소스 그냥 갖다쓰는 부분
}
