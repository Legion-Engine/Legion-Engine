

const int sampelePerTri=1;
const int K=30;
const uint RAND_MAX= 255;
#define maxPointsPerTri 300

#define radius 0.33f

#define cellSize (radius)/1.41421356237f
#define depth (int)ceil(1/ cellSize)


#define girdDimension  depth *depth

uint state = 777;
float floatState=0.777f;
uint Rand()
{
   state = state * 1664525 + 1013904223;
   return state >> 24;
}
float RandomValue()
{
   uint randInt =Rand();

   return randInt / (float) RAND_MAX;

}
uint wang_hash(uint seed)
{
        seed = (seed ^ 61) ^ (seed >> 16);
        seed *= 9;
        seed = seed ^ (seed >> 4);
        seed *= 0x27d4eb2d;
        seed = seed ^ (seed >> 15);
        return seed;
}
//includes upper bound
uint randomUpperRange(uint upper)
{
    uint r = Rand();
    return r %(upper+1);
}

float4 SampleTriangle(float2 rand,float4 a, float4 b, float4 c )
{
    float4 r;
    r= (float4)(a + rand.x* normalize(b-a) + rand.y * normalize(c-a));
    return r;
}
bool CheckPoint(float2 newPoint, float2* points, int* grid)
{
    //check if point lies within triangle range else just return false
    if(newPoint.x> 0 && newPoint.x <1 && newPoint.y>  0 && newPoint.y <1 && newPoint.x + newPoint.y < 1)
    {
        //check point validity
        //get grid cell
        int currentX = (int) newPoint.x / cellSize;
        int currentY = (int) newPoint.y / cellSize;
        //iterate neighbour cells in a radius of 2 
        int startX = max(0, currentX - 2);
        int endX = min(currentX + 2, maxPointsPerTri- 1);
        int startY = max(0, currentY - 2);
        int endY = min(currentY + 2, maxPointsPerTri-1);
        for(int x = startX; x < endX; x++)
        {
            for(int y = startY; y<endY; y++)
            {
                //get other cell, if valid, check distance is larger than radius
                int otherCellIndex = grid[x*y]-1;
                if(otherCellIndex>=0)
                {
                    float2 delta = (newPoint - points[otherCellIndex]);
                    float dist = sqrt(delta.x * delta.x + delta.y * delta.y);
                    if(dist< radius * radius) return false;
                }
            }
        }
        return true;
    }
    return false;
}
void PoissionSampling(__local float2* outputPoints)
{
    //init output
    //__local float2 outputPoints[maxPointsPerTri];
    int outPutIndex=0;
    //get celll size && cell count
    // float cellSize = radius/ sqrt(2.0f);
    // int dimX = ceil(dimensions.x / cellSize);
    // int dimY = ceil(dimensions.y / cellSize);
    //create cell grid
    __local int grid[maxPointsPerTri];

    __local float2 spawnPoints[maxPointsPerTri];
    // for(int i=0; i<maxPointsPerTri; i++)
    // spawnPoints[i]=0;
    //add center point as starting point
    spawnPoints[0] = (float2)(0.25f,0.25f);
    int spawnPointAmount=1;
    while(spawnPointAmount>0)
    {
       // spawnPointAmount--;
       //get random point from spawn points && its position
        int index = randomUpperRange(spawnPointAmount);
      //  float2 spawnCenter = spawnPoints[index-1];
        float2 spawnCenter = spawnPoints[spawnPointAmount];

        bool isAccepteed=false;
        //try generating a new valid point
        for(int i=0; i <K; i++)
        {
            float angle = RandomValue() * 2 * M_PI;
            float2 direction = (float2)(sin(angle), cos(angle));
            float2 newPoint = spawnCenter + normalize(direction) * radius;

            if(CheckPoint(newPoint,outputPoints, grid ))
            {
            outputPoints[outPutIndex] = newPoint;
            outPutIndex++;
            spawnPoints[spawnPointAmount] = newPoint;
            spawnPointAmount++;
            isAccepteed=true;

            grid[(int)(newPoint.x / cellSize) *(int)(newPoint.y/cellSize)] = outPutIndex;
            if(outPutIndex> sampelePerTri) return;
            }
        }
        if(!isAccepteed)
        {
        spawnPoints[index]=0;
        spawnPointAmount--;
        }     
    }

  //  return outputPoints;
}


__kernel void Main(__global const float* vertices,__global const uint* indices,__global const uint* seed,__global const float* randBuffer, __global float4* points)
{
    
    int n=get_global_id(0)*3;
  //  if(n>0) return;
    int currentSeed = seed[n];
    state= get_global_id(0);
    int resultIndex = get_global_id(0)*sampelePerTri;
    //get vertex indices
    uint vertex1Index = indices[n];
    uint vertex2Index = indices[n+1];
    uint vertex3Index = indices[n+2];

    //get vertex values
    //vertA
    float v1a = vertices[indices[n]*3];
    float v1b = vertices[indices[n]*3+1];
    float v1c = vertices[indices[n]*3+2];
    float4 vertA = (float4)(v1a,v1b,v1c,1.0f);
    //vert2
    float v2a = vertices[indices[n+1]*3];
    float v2b = vertices[indices[n+1]*3+1];
    float v2c = vertices[indices[n+1]*3+2];
    float4 vertB = (float4)(v2a,v2b,v2c,1.0f);
    //vert3
    float v3a = vertices[indices[n+2]*3];
    float v3b = vertices[indices[n+2]*3+1];
    float v3c = vertices[indices[n+2]*3+2];
    float4 vertC = (float4)(v3a,v3b,v3c,1.0f);


    __local float2 poissonOutput[maxPointsPerTri];
  //  uint r=0;

  //  poissonOutput =
   PoissionSampling(poissonOutput);
    for(int i =0; i <sampelePerTri; i++)
    {
         int index= resultIndex + i;
        float4 newPoint =SampleTriangle(poissonOutput[i],vertA,vertB,vertC);
     //   newPoint =(float4)( poissonOutput[i].x, poissonOutput[i].y,0,1);
        points[index]=newPoint;
      //  currentSeed++;
    }

 
}
