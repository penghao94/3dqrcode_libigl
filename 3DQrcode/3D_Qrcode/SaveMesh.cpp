#include "SaveMesh.h"

bool SaveMesh(const char* mesh_file_name,igl::viewer::Viewer viewer, igl::viewer::ViewerData data)
{
	std::string mesh_file_name_string(mesh_file_name);

	size_t last_dot = mesh_file_name_string.rfind('.');
	if (last_dot == std::string::npos)
	{
		// No file type determined
		printf("Error: No file extension found in %s\n", mesh_file_name);
		return false;
	}
	std::string extension = mesh_file_name_string.substr(last_dot + 1);
	if (extension == "off" || extension == "OFF" || extension == "obj" || extension == "OBJ")
	{
		return viewer.save_mesh_to_file(mesh_file_name);
	}
	else if (extension == "stl" || extension == "STL")
	{
		return igl::writeSTL(mesh_file_name_string, data.V, data.F, data.F_normals,true);
	}
	else
	{
		// unrecognized file type
		printf("Error: %s is not a recognized file type.\n", extension.c_str());
		return false;
	}
	return true;
}

