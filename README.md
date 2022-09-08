# chl_ceres_practice
현재 모듈화를 진행하지는 않았습니다. main함수에 함수를 구현해 놓은 상태입니다.  

detect : 특징점을 찾습니다.  
match : 두 이미지에서 검출된 특징점의 디스크립터를 비교하여 매칭합니다.  
computeEssentialMatrix : 매칭된 특징점으로 이미지간 Essential Matrix를 계산합니다.  
pixel2cam : 픽셀 좌표를 카메라 좌표로 전환합니다.    
computeTriangulation : 특징점과 R|t 매트릭스를 이용하여 삼각 측량을 수행합니다.   
SnavelyReprojectionError : 최적화를 위한 클래스입니다. Ceres 라이브러리에 포함된 템플릿 입니다.  
optimization : 최적화를 수행하는 함수입니다. 최적화된 R|t 매트릭스를 반환합니다.  
projection : 3D 좌표를 2D 좌표로 투영합니다. 투영된 좌표를 반환합니다.  
visualization : 결과를 OpenCV로 이미지화 합니다.  

## 22.09.08

Red : detected points  
Green : projected points  

[ befor optimization ]
![image](https://user-images.githubusercontent.com/58837749/189009270-95159723-61bb-4478-87c5-15332aeee951.png)  

[ after optimization ]
![Screenshot from 2022-09-08 09-24-03](https://user-images.githubusercontent.com/58837749/189008084-22c991e1-796d-4635-aeb2-b84cd8b4ce49.png)
![image](https://user-images.githubusercontent.com/58837749/189008360-6a31a3b7-4355-451a-b761-2594a4f67a10.png)


ceres-solver를 이용해서 최적화를 해보았다. 해당 이미지 상황은 KITTI 데이터에서 차량이 우회전 하는 상황이다.  
먼저 삼각측량을 통해서 구현된 3d point를 projection 했을 때에는, 약간의 오차가 존재하였지만 reprojection이 잘 되었다. 이를 optimization하고자 하였다.  
최적화 라이브러리는 ceres-solver를 사용하고, 최적화 방식은 doglet을 사용했는데, convergence가 떴는데도 오차가 매우 심하다. 원인이 무엇인지 잘 모르겠다. 어떻게 해결할 수 있을까?

## 22.09.09

```C++
    ceres::Problem problem;
    for (int i = 0; i < point2.size(); ++i)
    {
        double* point = new double[3]; // <-- 이 부분을 바깥으로 빼주니 결과가 달라지고 converge가 잘 되긴 한다. 하지만 최적화는 아직도 잘 이루어지지 않는 것 같다.
        point[0] = point_3d[i].x;
        point[1] = point_3d[i].y;
        point[2] = point_3d[i].z;
        ceres::CostFunction* cost_function =
            SnavelyReprojectionError::Create(
                point2[i].x,
                point2[i].y);
        problem.AddResidualBlock(cost_function,
                                new ceres::CauchyLoss(1.0),
                                camera,
                                point);
    }
///////////////////////////////////////////////////////////
    double* point = new double[3];
    for (int i = 0; i < point2.size(); ++i)
    {
        ...
    }
```

Three Views를 통해서 Optimization해보라는 의견에 구현을 하려고 보니(solvepnp를 사용하여), 결국엔 3D로 복원된 좌표와 마지막장에서 검출된 특징점과의 관계를 알아야만 solvepnp를 이용한 포즈 추정이 가능했다. 그래서 많은 SLAM들이 노드와 엣지를 구현하는 것 같다. 어떻게 구현해야할지...?
