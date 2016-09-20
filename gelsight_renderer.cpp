/****
GelSight OpenGL Renderer
Based heavily on lots of OpenGL code from around the internet
including learnopengl.com
****/

// Std. Includes
#include <string>
#include <unistd.h>

// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// GL includes
#include "shader.h"
#include "camera.h"
#include "mesh.h"

// GLM Mathemtics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/image_t.hpp>

#include <opencv2/opencv.hpp>


// Properties
GLuint screenWidth = 800, screenHeight = 600;

// Function prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void Do_Movement();

// Camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
bool keys[1024];
GLfloat lastX = 400, lastY = 300;
bool firstMouse = true;

GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

Mesh * gs_mesh;

// Is called whenever a key is pressed/released via GLFW
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);

    if(action == GLFW_PRESS)
        keys[key] = true;
    else if(action == GLFW_RELEASE)
        keys[key] = false;  
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if(firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    GLfloat xoffset = xpos - lastX;
    GLfloat yoffset = lastY - ypos; 
    
    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
} 

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}

void handleGelsightFrameMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::image_t* msg, void * state) {

  // gelsight frame comes in as an image
  bool success = false;
  cv::Mat decodedImage = cv::imdecode(msg->data, 0);
  if (decodedImage.rows > 0){

    int num_verts_needed = decodedImage.cols * decodedImage.rows;
    int num_inds_needed = (decodedImage.cols - 1) * (decodedImage.rows - 1) * 3 * 2;

    std::vector<Vertex> new_verts;
    new_verts.resize(num_verts_needed);
    std::vector<GLuint> new_inds;
    new_inds.resize(num_inds_needed);
    std::vector<Texture> new_textures;

    int ind_vert = 0;
    for (int i = 0; i < decodedImage.cols; i++){
      for (int j = 0; j < decodedImage.rows; j++){
        unsigned char depth_char = decodedImage.at<unsigned char>(j,i);
        float depth = ((float)depth_char)/255.0;
        new_verts[ind_vert].Position.x = ((((float)i)/(float)decodedImage.cols))*5.0 - 2.5;
        new_verts[ind_vert].Position.y = ((((float)j)/(float)decodedImage.rows))*5.0 - 2.5;
        new_verts[ind_vert].Position.z = (depth*5.0)*0.1;

        new_verts[ind_vert].TexCoords.x = ((((float)i)/(float)decodedImage.cols));
        new_verts[ind_vert].TexCoords.y = ((((float)j)/(float)decodedImage.rows));

        ind_vert++;
      }
    }

    ind_vert = 0;
    for (int i = 0; i < decodedImage.cols; i++){
      for (int j = 0; j < decodedImage.rows; j++){
        int right_vert = (ind_vert + decodedImage.rows) % num_verts_needed;
        int down_vert = (ind_vert + 1) % num_verts_needed;

        float depth = new_verts[ind_vert].Position.z;
        float right_depth = new_verts[right_vert].Position.z;
        float down_depth = new_verts[down_vert].Position.z;

        new_verts[ind_vert].Normal.x = right_depth - depth;
        new_verts[ind_vert].Normal.y = down_depth - depth;
        new_verts[ind_vert].Normal.z = 1.0;
        new_verts[ind_vert].Normal = glm::normalize(new_verts[ind_vert].Normal);

        new_verts[ind_vert].Tangent.x = 1.0;
        new_verts[ind_vert].Tangent.y = 0.0;
        new_verts[ind_vert].Tangent.z = 0.0;

        new_verts[ind_vert].Bitangent.x = 0.0;
        new_verts[ind_vert].Bitangent.y = 1.0;
        new_verts[ind_vert].Bitangent.z = 0.0;

        ind_vert++;
      }
    }

    int ind_ind = 0;    
    for (int i = 0; i < decodedImage.cols - 1; i++){
      for (int j = 0; j < decodedImage.rows - 1; j++){
        new_inds[ind_ind + 0] = i*(decodedImage.rows) + j;
        new_inds[ind_ind + 2] = i*(decodedImage.rows) + j+1;
        new_inds[ind_ind + 1] = (i+1)*(decodedImage.rows) + j;

        new_inds[ind_ind + 3] = i*(decodedImage.rows) + j + 1;
        new_inds[ind_ind + 5] = (i+1)*(decodedImage.rows) + j+1;
        new_inds[ind_ind + 4] = (i+1)*(decodedImage.rows) + j;

        ind_ind += 6;
      }
    }

    gs_mesh->Reset(new_verts, new_inds, new_textures);

  } else {
    printf("Trouble decoding incoming gelsight depth im\n");
  }
}

// Moves/alters the camera positions based on user input
void Do_Movement()
{
    // Camera controls
    if(keys[GLFW_KEY_W])
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if(keys[GLFW_KEY_S])
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if(keys[GLFW_KEY_A])
        camera.ProcessKeyboard(LEFT, deltaTime);
    if(keys[GLFW_KEY_D])
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

void *lcmMonitor(void *plcm) {
  lcm::LCM * lcm = (lcm::LCM *) plcm;
  while (1){
    lcm->handle();
    usleep(500);
  }
}

// The MAIN function, from here we start our application and run our Game loop
int main()
{
  printf("init\n");
  lcm::LCM lcm;

  // Init GLFW
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "LearnOpenGL", NULL, NULL); // Windowed
  glfwMakeContextCurrent(window);

  // Set the required callback functions
  glfwSetKeyCallback(window, key_callback);
  glfwSetCursorPosCallback(window, mouse_callback);
  glfwSetScrollCallback(window, scroll_callback);

  // Options
  //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // Initialize GLEW to setup the OpenGL Function pointers
  glewExperimental = GL_TRUE;
  glewInit();

  // Define the viewport dimensions
  glViewport(0, 0, screenWidth, screenHeight);

  // Setup some OpenGL options
  glEnable(GL_DEPTH_TEST);

  // Setup and compile our shaders
  Shader shader("shader.vs", "shader.frag");

  // Load models
  gs_mesh = new Mesh;

  lcm.subscribeFunction("GELSIGHT_DEPTH", &handleGelsightFrameMsg, (void *) NULL);
  
  pthread_t lcmThread;
  pthread_create(&lcmThread, NULL, lcmMonitor, &lcm);

  // Draw in wireframe
  //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // Game loop
  while(!glfwWindowShouldClose(window))
  {
      // Set frame time
      GLfloat currentFrame = glfwGetTime();
      deltaTime = currentFrame - lastFrame;
      lastFrame = currentFrame;

      // Check and call events
      glfwPollEvents();
      Do_Movement();

      // Clear the colorbuffer
      glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      shader.Use();   // <-- Don't forget this one!
      // Transformation matrices
      glm::mat4 projection = glm::perspective(camera.Zoom, (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
      glm::mat4 view = camera.GetViewMatrix();
      glUniformMatrix4fv(glGetUniformLocation(shader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
      glUniformMatrix4fv(glGetUniformLocation(shader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));

      GLint objectColorLoc = glGetUniformLocation(shader.Program, "objectColor");
      GLint lightColorLoc  = glGetUniformLocation(shader.Program, "lightColor");
      GLint lightPosLoc    = glGetUniformLocation(shader.Program, "lightPos");
      GLint viewPosLoc     = glGetUniformLocation(shader.Program, "viewPos");
      glUniform3f(objectColorLoc, 1.0f, 0.5f, 0.31f);
      glUniform3f(lightColorLoc,  1.0f, 1.0f, 1.0f);
      glUniform3f(lightPosLoc,    10.0, 10.0, 10.0);
      glUniform3f(viewPosLoc,     camera.Position.x, camera.Position.y, camera.Position.z);

      // Draw the loaded model
      glm::mat4 model;
      //model = glm::translate(model, glm::vec3(0.0f, -0.0f, -10.0f)); // Translate it down a bit so it's at the center of the scene
      // model = glm::scale(model, glm::vec3(0.2f, 0.2f, 0.2f)); // It's a bit too big for our scene, so scale it down
      glUniformMatrix4fv(glGetUniformLocation(shader.Program, "model"), 1, GL_FALSE, glm::value_ptr(model));
      gs_mesh->Draw(shader);       

      // Swap the buffers
      glfwSwapBuffers(window);
      usleep(10);
  }

  glfwTerminate();
  return 0;
}