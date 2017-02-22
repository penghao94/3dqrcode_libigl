#include "Select.h"

bool qrcode::select(igl::viewer::Viewer& viewer, Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C)
{
	int fid;
	Eigen::Vector3f bc;
	// Cast a ray in the view direction starting from the mouse position
	double x = viewer.current_mouse_x;
	double y = viewer.core.viewport(3) - viewer.current_mouse_y;

	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view * viewer.core.model,
		viewer.core.proj, viewer.core.viewport, V, F, fid, bc))
	{
		// paint hit red
		C.row(fid) << 1, 0, 0;
		cout << fid << endl;
		viewer.data.set_colors(C);
		return true;
	}
	return false;
}
