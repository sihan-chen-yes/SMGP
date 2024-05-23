# Assignment 5
Name: 'Sihan Chen'

Legi-Nr: '23-943-079'

## Required results
Edit this 'README.md' file to answer the theory question(s).

### Tasks

1) Compare the results of the multi-resolution mesh editing and the deformation transfer. What are the differences between the two methods? What are the advantages and disadvantages of each method?

## Reports

### Multi-resolution mesh editing v.s. Deformation transfer

#### Observations

|      | High-freq detail transfer             | Deformation transfer                 |
| :-----------:  | ------------------------------------- |------------------------------------- |
| Results | ![image-20240515190143948](./README.assets/image-20240515190143948.png) | ![image-20240515190158995](./README.assets/image-20240515190158995.png) |
| cylinder | ![image-20240515185525546](./README.assets/image-20240515185525546.png) | ![image-20240515185516325](./README.assets/image-20240515185516325.png) |
| bar | ![image-20240523200302211](./README.assets/image-20240523200302211.png) | ![image-20240523200320354](./README.assets/image-20240523200320354.png) |
| cactus | ![image-20240523200449034](./README.assets/image-20240523200449034.png) | ![image-20240523200504390](./README.assets/image-20240523200504390.png) |
| camel_head | ![image-20240523200642100](./README.assets/image-20240523200642100.png) | ![image-20240523200714041](./README.assets/image-20240523200714041.png) |
| hand | ![image-20240523201135692](./README.assets/image-20240523201135692.png) | ![image-20240523201149871](./README.assets/image-20240523201149871.png) |
| woody-hi | ![image-20240523201301130](./README.assets/image-20240523201301130.png) | ![image-20240523201312706](./README.assets/image-20240523201312706.png) |
| woody-lo | ![image-20240523201423884](./README.assets/image-20240523201423884.png) | ![image-20240523201436312](./README.assets/image-20240523201436312.png) |
|  Differences  | main idea is to encode the high-frequency details into rotation invariant representation, then use naive-laplacian mesh editing and add the details back | equivalence to gradient-based deformations, main idea is to transfer the deformation between low-frequency mesh onto original mesh via solving Poisson system |
|  Advantages   | fast computation, just need to add a displacement for every vertices | able to handle mesh with a obvious height field              |
| Disadvantages | 1.not aware of the whole mesh, just add the coordinates separately leading to self-intersections of mesh 2.sometimes can't preserve details of mesh | higher computational complexity, need to compute the Sj and solve the Poisson system every time |
