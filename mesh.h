#pragma once

/* Used from https://github.com/JoeyDeVries/LearnOpenGL/blob/master/includes/learnopengl/shader.h */


// Std. Includes
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
using namespace std;
// GL Includes
#include <GL/glew.h> // Contains all the necessery OpenGL includes
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


struct Vertex {
    // Position
    glm::vec3 Position;
    // Normal
    glm::vec3 Normal;
    // TexCoords
    glm::vec2 TexCoords;
    // Tangent
    glm::vec3 Tangent;
    // Bitangent
    glm::vec3 Bitangent;
};

struct Texture {
    GLuint id;
    string type;
    string path;
};

class Mesh {
public:
    /*  Mesh Data  */
    vector<Vertex> vertices_;
    vector<GLuint> indices_;
    vector<Texture> textures_;
    GLuint VAO;

    /*  Functions  */
    // Constructor
    Mesh()
    {
        // Create buffers/arrays
        glGenVertexArrays(1, &this->VAO);
        glGenBuffers(1, &this->VBO);
        glGenBuffers(1, &this->EBO);
    }

    void Reset(vector<Vertex> vertices, vector<GLuint> indices, vector<Texture> textures)
    {
        this->vertices_ = vertices;
        this->indices_ = indices;
        this->textures_ = textures;
    }

    // Render the mesh
    void Draw(Shader shader) 
    {
        // Bind appropriate textures
        GLuint diffuseNr = 1;
        GLuint specularNr = 1;
        GLuint normalNr = 1;
        GLuint heightNr = 1;
        for(GLuint i = 0; i < this->textures_.size(); i++)
        {
            glActiveTexture(GL_TEXTURE0 + i); // Active proper texture unit before binding
            // Retrieve texture number (the N in diffuse_textureN)
            stringstream ss;
            string number;
            string name = this->textures_[i].type;
            if(name == "texture_diffuse")
                ss << diffuseNr++; // Transfer GLuint to stream
            else if(name == "texture_specular")
                ss << specularNr++; // Transfer GLuint to stream
            else if(name == "texture_normal")
                ss << normalNr++; // Transfer GLuint to stream
             else if(name == "texture_height")
                ss << heightNr++; // Transfer GLuint to stream
            number = ss.str(); 
            // Now set the sampler to the correct texture unit
            glUniform1i(glGetUniformLocation(shader.Program, (name + number).c_str()), i);
            // And finally bind the texture
            glBindTexture(GL_TEXTURE_2D, this->textures_[i].id);
        }
        
        if (this->vertices_.size() > 0){
            // Draw mesh

            glBindVertexArray(this->VAO);
            // Load data into vertex buffers
            glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
            // A great thing about structs is that their memory layout is sequential for all its items.
            // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
            // again translates to 3/2 floats which translates to a byte array.
            glBufferData(GL_ARRAY_BUFFER, this->vertices_.size() * sizeof(Vertex), &this->vertices_[0], GL_STATIC_DRAW);  

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->EBO);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices_.size() * sizeof(GLuint), &this->indices_[0], GL_STATIC_DRAW);

            // Set the vertex attribute pointers
            // Vertex Positions
            glEnableVertexAttribArray(0);   
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
            // Vertex Normals
            glEnableVertexAttribArray(1);   
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, Normal));
            // Vertex Texture Coords
            glEnableVertexAttribArray(2);   
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, TexCoords));
            // Vertex Tangent
            glEnableVertexAttribArray(3);
            glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, Tangent));
            // Vertex Bitangent
            glEnableVertexAttribArray(4);
            glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, Bitangent));

            glBindVertexArray(0);

            printf("trying to draw one\n");

            glBindVertexArray(this->VAO);
            glDrawElements(GL_TRIANGLES, this->indices_.size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }

        // Always good practice to set everything back to defaults once configured.
        for (GLuint i = 0; i < this->textures_.size(); i++)
        {
            glActiveTexture(GL_TEXTURE0 + i);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }

private:
    /*  Render data  */
    GLuint VBO, EBO;
};
