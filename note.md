- 光栅化成像
- 几何表示
- 光的传播理论
- 动画与模拟

## 变换

目的：把三维世界看到的影像投影到相机的二维幕布上

变换矩阵 $M = M_{vp} M_{proj} M_{cam}$

1. The Camera Transformation：通过变换$M_{cam}$，把相机的坐标和方位规范化，相机的坐标为原点，+Y为向上，-Z为看向的方向。并将变换$M_{cam}$施加到世界中的所有物体上。

   ![](./img/M_cam.jpeg)

2. The Projection Transformation：通过变换$M_{proj}$把3D空间压缩到原点的$[-1, 1]^3$规范空间内，xy轴保留着物体上下左右的关系，z轴保留了深度（远近）的信息

   ![](./img/M_proj.jpeg)

3. The Viewport Transformation：通过变换$M_{vp}$，把规范空间内的3D空间，拍摄为-Z方向的2D图像

   ![](./img/M_vp.jpeg)

> *Model View Projection*
>
> - Model：世界中的物体
> - View：相机
> - Projection：投影方法
>   - Orthographic Projection 正交投影
>   - Perspective Projection 透视投影

