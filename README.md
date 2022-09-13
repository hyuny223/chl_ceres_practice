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

## 22.09.13
[ before optimization ]
![image](https://user-images.githubusercontent.com/58837749/189782596-36041386-7621-4ae2-9247-870ba448efa9.png)  

[ after optimization ]
![image](https://user-images.githubusercontent.com/58837749/189782701-5f195eba-202f-4a4c-83cd-e257d3621aab.png)  

이번에 사용한 방식은 solvePnPRANSAC이다.  
1) 1번, 2번 이미지에서 SIFT를 이용하여 코너를 검출하고, 매칭을 시행한다.  
2) 매칭점을 이용하여 Essential Matrix를 구한다.  
3) 구한 E를 이용하여 R|t를 복원한다.  
4) 복원한 R|t를 이용하여 Triangulation을 진행한다.  
5) 이를 FramePoint로 만들어준다.  
6) 이미 구해진 2번 이미지의 키포인트와, SIFT를 이용하여 구해진 3번 이미지의 키포인트를 매칭시킨다.  
7) 이 때 중요한 점은, 3번 프레임과의 매칭점 중, 2번 이미지와 1번 이미지에서 모두 발견된 것을 골라내는 것이 중요하다. 그래야만 3D 포인트를 사용할 수 있다. 여기서는 unordered_map을 이용하여 query, train index 정보를 키값으로 저장하여, 이미 저장된 값이 있다면 이전 프레임과 연결되어 있는 것으로 판단하여 그때의 3D 포인트 값과, 3번 이미지에서의 2D값을 저장하여 반환한다.  
8) 이렇게 구해진 3D값과 2D 값을 이용하여 solvePnPRANSAC을 수행한다.  
9) R|t가 구해지면 projection을 수행한다.  
10) optimization을 수행한다.  
11) 최적화된 R|t가 구해지면 reprojection을 수행하여 결과를 비교한다.  
  
여기까지가 수행한 단계인데, 결과가 좋지 못한 것 같다. 무엇이 문제일까? 분명히 최적화를 방해하는 outliers가 존재하는 것 같은데, 예를 들어 최적화 전 projection의 결과에 마이너스 값이 나오면 필터링 해주는 것이 맞을까? solvepnp가 정확하다는 가정 아래, 마이너스 값이 나오면 안 된다.

아래는 9번 과정의 결과이다.  
```bash
origin : (22.788500, 230.006882)
projec : (590.205214, -28.773294)
=================
origin : (23.407982, 259.927979)
projec : (606.207165, -32.218455)
=================
origin : (70.747231, 308.567810)
projec : (590.978335, 50.718804)
=================
origin : (83.044502, 309.850800)
projec : (585.276824, 57.971245)
=================
origin : (112.412354, 299.959747)
projec : (609.899974, 15.830458)
=================
origin : (123.777145, 214.815445)
projec : (611.707643, 139.005742)
=================
origin : (146.008469, 144.284180)
projec : (639.182287, 140.319481)
=================
origin : (162.140259, 143.838455)
projec : (661.001986, -55.340999)
=================
origin : (139.200409, 285.879211)
projec : (684.175607, 67.605407)
=================
origin : (144.634277, 287.814850)
projec : (697.325105, -13.790014)
=================
origin : (168.586548, 131.631851)
projec : (726.400629, -42.894110)
=================
origin : (175.471634, 275.491516)
projec : (761.611304, -43.565234)
=================
origin : (183.147522, 272.431610)
projec : (822.458695, 26.432818)
=================
origin : (217.037674, 240.904297)
projec : (848.809171, 89.596108)
=================
origin : (246.826843, 243.948456)
projec : (851.619501, 76.445709)
=================
origin : (299.345520, 246.043884)
projec : (852.588645, 39.251664)
=================
origin : (310.861359, 198.396576)
projec : (851.984296, 121.154142)
=================
origin : (320.710175, 212.926712)
projec : (885.984746, -7.561131)
=================
origin : (329.284882, 79.159042)
projec : (888.623114, 3.573617)
=================
origin : (346.112030, 91.790794)
projec : (871.608026, 122.891977)
=================
origin : (353.943848, 184.485550)
projec : (874.888399, 93.481893)
=================
origin : (355.049744, 188.473160)
projec : (883.389207, -45.061986)
=================
origin : (365.173584, 99.883728)
projec : (911.431005, 120.343448)
=================
origin : (373.549377, 186.829803)
projec : (923.298345, 21.510419)
=================
origin : (390.656952, 190.556992)
projec : (928.099091, 119.196268)
=================
origin : (394.305328, 164.332336)
projec : (944.684881, 74.358661)
=================
origin : (374.954681, 343.772644)
projec : (950.356025, 53.317886)
=================
origin : (414.922302, 160.546921)
projec : (949.715524, 78.884759)
=================
origin : (433.033081, 120.791275)
projec : (949.589997, 117.903777)
=================
origin : (444.403687, 162.238190)
projec : (952.623403, 212.494818)
=================
origin : (448.498138, 159.938766)
projec : (959.707096, 60.484122)
=================
origin : (445.887421, 91.722435)
projec : (959.954769, 90.355036)
=================
origin : (448.078033, 131.127777)
projec : (960.307541, 40.068695)
=================
origin : (446.466431, 87.855560)
projec : (963.660989, 15.583593)
=================
origin : (449.577057, 146.863098)
projec : (974.890334, -19.423956)
=================
origin : (449.436096, 98.050919)
projec : (973.339674, 52.878325)
=================
origin : (460.471649, 125.383217)
projec : (977.322683, 86.597054)
=================
origin : (500.349609, 133.845200)
projec : (979.106628, 81.713593)
=================
origin : (501.714478, 244.943481)
projec : (982.335849, 72.285876)
=================
origin : (525.537659, 194.562042)
projec : (992.238608, 88.980394)
=================
origin : (523.962036, 243.334259)
projec : (999.390701, -22.855780)
=================
origin : (523.962036, 243.334259)
projec : (1016.817054, 199.167062)
=================
origin : (534.155579, 36.601719)
projec : (1002.499926, 6.062514)
=================
origin : (534.216492, 42.835518)
projec : (1007.241384, 90.452512)
=================
origin : (526.328796, 312.671051)
projec : (1005.897817, 63.266508)
=================
origin : (532.655273, 253.474808)
projec : (1009.125679, 71.075490)
=================
origin : (534.042236, 242.357941)
projec : (1016.704598, 57.606881)
=================
origin : (547.350220, 24.675337)
projec : (1016.704598, 57.606881)
=================
origin : (548.575989, 44.063225)
projec : (1019.971928, 70.379438)
=================
```
  
아래는 11번 과정의 결과이다.  
```bash
origin : (22.788500, 230.006882)
projec : (74.302020, 122.136727)o
=================
origin : (23.407982, 259.927979)
projec : (78.027858, 121.660445)o
=================
origin : (70.747231, 308.567810)
projec : (223.281129, 142.599261)o
=================
origin : (83.044502, 309.850800)
projec : (273.112253, 124.778449)o
=================
origin : (112.412354, 299.959747)
projec : (165.299505, 136.251875)o
=================
origin : (123.777145, 214.815445)
projec : (274.198692, 225.581457)o
=================
origin : (146.008469, 144.284180)
projec : (273.383860, 234.514489)o
=================
origin : (162.140259, 143.838455)
projec : (73.455859, 115.349408)o
=================
origin : (139.200409, 285.879211)
projec : (263.971904, 168.747981)o
=================
origin : (144.634277, 287.814850)
projec : (138.029196, 142.779473)o
=================
origin : (168.586548, 131.631851)
projec : (173.484013, 101.542242)o
=================
origin : (175.471634, 275.491516)
projec : (186.273160, 105.380318)o
=================
origin : (183.147522, 272.431610)
projec : (290.254739, 147.785199)o
=================
origin : (217.037674, 240.904297)
projec : (334.544680, 199.772263)o
=================
origin : (246.826843, 243.948456)
projec : (333.128741, 187.236622)o
=================
origin : (299.345520, 246.043884)
projec : (320.414116, 154.375319)o
=================
origin : (310.861359, 198.396576)
projec : (348.385324, 226.934651)o
=================
origin : (320.710175, 212.926712)
projec : (-129.709299, 335.886651)o
=================
origin : (329.284882, 79.159042)
projec : (-134.159850, 350.653388)o
=================
origin : (346.112030, 91.790794)
projec : (369.401088, 222.667630)o
=================
origin : (353.943848, 184.485550)
projec : (362.278702, 196.179954)o
=================
origin : (355.049744, 188.473160)
projec : (295.939018, 85.389184)o
=================
origin : (365.173584, 99.883728)
projec : (411.251279, 207.875329)o
=================
origin : (373.549377, 186.829803)
projec : (78.980165, 273.080341)o
=================
origin : (390.656952, 190.556992)
projec : (426.058128, 202.899841)o
=================
origin : (394.305328, 164.332336)
projec : (428.736864, 159.344367)o
=================
origin : (374.954681, 343.772644)
projec : (433.796114, 136.636806)o
=================
origin : (414.922302, 160.546921)
projec : (462.236290, 148.453762)o
=================
origin : (433.033081, 120.791275)
projec : (447.028655, 195.697161)o
=================
origin : (444.403687, 162.238190)
projec : (1308.746510, -139.536378)o
=================
origin : (448.498138, 159.938766)
projec : (443.395805, 141.338122)o
=================
origin : (445.887421, 91.722435)
projec : (449.450738, 168.739580)o
=================
origin : (448.078033, 131.127777)
projec : (244.409722, 219.349528)o
=================
origin : (446.466431, 87.855560)
projec : (430.242496, 103.049131)o
=================
origin : (449.577057, 146.863098)
projec : (429.906482, 70.317703)o
=================
origin : (449.436096, 98.050919)
projec : (378.293667, 169.047768)o
=================
origin : (460.471649, 125.383217)
projec : (462.319311, 162.324760)o
=================
origin : (500.349609, 133.845200)
projec : (462.000991, 157.945586)o
=================
origin : (501.714478, 244.943481)
projec : (464.673565, 147.820266)o
=================
origin : (525.537659, 194.562042)
projec : (479.376311, 159.459755)o
=================
origin : (523.962036, 243.334259)
projec : (451.552473, 62.322906)o
=================
origin : (523.962036, 243.334259)
projec : (1221.949888, -97.961350)o
=================
origin : (534.155579, 36.601719)
projec : (464.766666, 85.714894)o
=================
origin : (534.216492, 42.835518)
projec : (600.544219, 103.840196)o
=================
origin : (526.328796, 312.671051)
projec : (483.077194, 134.830312)o
=================
origin : (532.655273, 253.474808)
projec : (490.829520, 139.492904)o
=================
origin : (534.042236, 242.357941)
projec : (495.117384, 125.586032)o
=================
origin : (547.350220, 24.675337)
projec : (495.117384, 125.586032)o
=================
origin : (548.575989, 44.063225)
projec : (500.805361, 136.196921)o
=================
```
