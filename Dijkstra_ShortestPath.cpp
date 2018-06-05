//==================================================================================================================
// Name        : ShortestPath.cpp
// Author      : Yamile Vargas
// Copyright   : All rights reserved / Heap code: Advance data Structures - Peter Brass
// Description : Find the shortest path between two points in a plane with obstacles using Dijkstra's Algorithm.
//==================================================================================================================

#include <algorithm>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stack>
#include <queue>
#include <stdlib.h>
using namespace std;

#define NUMBER_POINTS 3002
#define SOURCE 0
#define TARGET 3001

	FILE *fp;

    /*Points Structure*/
    typedef struct st_point {
  	  double x;
  	  double y;
  	  int id;
    	} point;

    int triangles_graph[NUMBER_POINTS][NUMBER_POINTS] =  {};
    double graph[NUMBER_POINTS][NUMBER_POINTS] =  {};

    int num_triangles;
    point points[NUMBER_POINTS] = {};

    //evaluated[]: keeps track of the nodes that has been evaluated as visible points
    bool evaluated[NUMBER_POINTS] = {};
    queue <int> index_;
    bool shortest_path_found = false;
    stack <point> shortest_path;
    int point_counter = 1;

    stack <point> shortest_path_dijkstras();

/*********************************WINDOW DISPLAY: *********************************/
    Display *display_ptr;
    Screen *screen_ptr;
    int screen_num;
    char *display_name = NULL;
    unsigned int display_width, display_height;

    Window win;
    int border_width;
    unsigned int win_width, win_height;
    int win_x, win_y;

    XWMHints *wm_hints;
    XClassHint *class_hints;
    XSizeHints *size_hints;
    XTextProperty win_name, icon_name;
    char *win_name_string = "Shortest path - Jvargas";
    char *icon_name_string = "Icon for Example Window";

    XEvent report;

    GC gc, gc_delete, gc_red, gc_grey;
    unsigned long valuemask = 0;
    XGCValues gc_values, gc_white_values, gc_red_values, gc_grey_values;
    Colormap color_map;
    XColor tmp_color1, tmp_color2;


/*----Finding the graph--- new edges-----------------------------------------*/

	int s_t = 0;
	int counter_right_click=0;
	int count_points = 1;
	int points_in_triangule =0;

	/*intersect: Evaluate if the given edges/lines intersect by determinant method.
	 * Existent edge from p to q.
	 * Possible edge from r to s
	 * returns true if the possible edge rs intersects pq.
	 * and false otherwise
	 */
	bool intersect(point s, point r, point p, point q){
		double   orientations_pqr;
		double   orientations_pqs;
		double   orientations_rsp;
		double   orientations_rsq;

		double result_pq;
		double result_rs;

		orientations_pqr = ((p.x * q.y) + (p.y * r.x) + (q.x * r.y)) - ((p.x * r.y) + (q.x * p.y) + (r.x * q.y) );
		orientations_pqs = ((p.x * q.y) + (p.y * s.x) + (q.x * s.y)) - ((p.x * s.y) + (q.x * p.y) + (s.x * q.y) );
		result_pq = orientations_pqr * orientations_pqs;

		orientations_rsp = ((r.x * s.y) + (r.y * p.x) + (s.x * p.y)) - ((r.x * p.y) + (s.x * r.y) + (p.x * s.y) );
		orientations_rsq = ((r.x * s.y) + (r.y * q.x) + (s.x * q.y)) - ((r.x * q.y) + (s.x * r.y) + (q.x * s.y) );
		result_rs = orientations_rsp * orientations_rsq;

		return ((result_pq < 0) and (result_rs < 0));
	}

	/*next_p: returns the coordinates of the point p in the array of points
	 * for the next existing edge of a triangle*/
	int next_p = 1;

	/*next_q: returns the coordinates of the point q in the array of points
	 * for the next existing edge of a triangle*/
	int next_q(int p, int q){
		int m=0;
		q++;
		int q_ = q;
		int next_ =-1;
		if ((q_ >= point_counter) and (p < point_counter -1)){
			p++;
			q_ = 1;
			next_p = p;
		}
		for (m = q_; m < point_counter; m ++){
			if (triangles_graph[p][m] == 1){
				next_ = m;
				q_= point_counter;
				m = point_counter;
			}
			else if ((m == point_counter - 1) and (p < point_counter-1)) {
				p++;
				q_ = 1;
				m = 0;
				next_p = p;
			}
		}
		return next_;
	}

	/* new_edge:
     * returns 0: when there exist at least an edge p-q of the set of triangles thta intersects s-r
	 * returns 1: there is no edge that intersects s-r 
     * When q is -1 there are no more edges to compare with. */
	int new_edge( point s, point r, int q){

		int nextq = 0;
		int result = 1;
		/*when s==r it is not a valid edge */
		if ( (s.x == r.x) and (s.y == r.y))
			result = 0;
		else if ( q == -1){
			next_p = 0;
			result = 1;
		}else if (intersect(s,r, points[next_p], points[q])){
			next_p = 0;
			result = 0;
		}else{
			nextq = next_q(next_p,q);
			result = new_edge(s, r, nextq);
		}
		return result;
	}

	/* inside_triangle_ function evaluates if the point r is inside triangle p */
	bool inside_triangle_(point p1, point r){
		point  p, q, s;
		bool q1, q2, q3;
		bool is_inside = false;

		if(p1.id != r.id and p1.id != TARGET){
			s = points[p1.id*3];
			q = points[(p1.id*3)-1];
			p = points[(p1.id*3)-2];

			q1 = (((((p.x * q.y) + (p.y * r.x) + (q.x * r.y)) - ((p.x * r.y) + (q.x * p.y) + (r.x * q.y) ))*
					(((p.x * s.y) + (p.y * r.x) + (s.x * r.y)) - ((p.x * r.y) + (s.x * p.y) + (r.x * s.y) )))<0);
			q2 = (((((q.x * p.y) + (q.y * r.x) + (p.x * r.y)) - ((q.x * r.y) + (p.x * q.y) + (r.x * p.y) ))*
					(((q.x * s.y) + (q.y * r.x) + (s.x * r.y)) - ((q.x * r.y) + (s.x * q.y) + (r.x * s.y) )))<0);
			q3 = (((((s.x * p.y) + (s.y * r.x) + (p.x * r.y)) - ((s.x * r.y) + (p.x * s.y) + (r.x * p.y) ))*
					(((s.x * q.y) + (s.y * r.x) + (q.x * r.y)) - ((s.x * r.y) + (q.x * s.y) + (r.x * q.y) )))<0);

			is_inside = q1 and q2 and q3;

			s = points[r.id*3];
			q = points[(r.id*3)-1];
			p = points[(r.id*3)-2];

			q1 = (((((p.x * q.y) + (p.y * p1.x) + (q.x * p1.y)) - ((p.x * p1.y) + (q.x * p.y) + (p1.x * q.y) ))*
					(((p.x * s.y) + (p.y * p1.x) + (s.x * p1.y)) - ((p.x * p1.y) + (s.x * p.y) + (p1.x * s.y) )))<0);
			q2 = (((((q.x * p.y) + (q.y * p1.x) + (p.x * p1.y)) - ((q.x * p1.y) + (p.x * q.y) + (p1.x * p.y) ))*
					(((q.x * s.y) + (q.y * p1.x) + (s.x * p1.y)) - ((q.x * p1.y) + (s.x * q.y) + (p1.x * s.y) )))<0);
			q3 = (((((s.x * p.y) + (s.y * p1.x) + (p.x * p1.y)) - ((s.x * p1.y) + (p.x * s.y) + (p1.x * p.y) ))*
					(((s.x * q.y) + (s.y * p1.x) + (q.x * p1.y)) - ((s.x * p1.y) + (q.x * s.y) + (p1.x * q.y) )))<0);

			if(q1 and q2 and q3)
				is_inside = true;
		}
		return is_inside;
	}

	/* inside_triangle function evaluates if the point r is inside p triangle*/
	bool inside_triangle(point p1, point r){
		point  p, q, s;
		bool q1, q2, q3;
		bool is_inside = false;

		if(p1.id != r.id and p1.id != TARGET){
			s = points[p1.id*3];
			q = points[(p1.id*3)-1];
			p = points[(p1.id*3)-2];

			q1 = (((((p.x * q.y) + (p.y * r.x) + (q.x * r.y)) - ((p.x * r.y) + (q.x * p.y) + (r.x * q.y) ))*
					(((p.x * s.y) + (p.y * r.x) + (s.x * r.y)) - ((p.x * r.y) + (s.x * p.y) + (r.x * s.y) )))<0);
			q2 = (((((q.x * p.y) + (q.y * r.x) + (p.x * r.y)) - ((q.x * r.y) + (p.x * q.y) + (r.x * p.y) ))*
					(((q.x * s.y) + (q.y * r.x) + (s.x * r.y)) - ((q.x * r.y) + (s.x * q.y) + (r.x * s.y) )))<0);
			q3 = (((((s.x * p.y) + (s.y * r.x) + (p.x * r.y)) - ((s.x * r.y) + (p.x * s.y) + (r.x * p.y) ))*
					(((s.x * q.y) + (s.y * r.x) + (q.x * r.y)) - ((s.x * r.y) + (q.x * s.y) + (r.x * q.y) )))<0);

			is_inside = q1 and q2 and q3;
		}
		return is_inside;
	}

	/*Distance between two points*/
	double distance_p(point p1, point p2){
		double l = (sqrt( pow( p2.x - p1.x, 2 ) + pow(p2.y - p1.y, 2 )));
		return l;
	}

	/*  To build the graph we evaluate every possible edge (new or existing)
	 *  against all other existing edges, if no edge intersects it then it
	 *  is a good edge.
	 *  */
	void build_graph_triangles(){

		int nextq = 0;
		int l=0;
		int temp_i = 1;
		/*This loop looks for the visible points from the source*/
		for (temp_i = 1; temp_i < point_counter-1; temp_i++){
				for(l= temp_i + 1; l < point_counter; l++){
						nextq = next_q(next_p,0);
						if (new_edge(points[temp_i], points[l], nextq) == 1){
							if(!inside_triangle_(points[temp_i],points[l]))
								graph[temp_i][l] = distance_p(points[temp_i],points[l]);
						}
					}
		}
	}

	/*  Finds the visible points from a given point
	 *  */
	bool visible_points(int p, bool target_){
		next_p = 1;
		int nextq = 0;
		int l=0;
		bool is_inside_a_triangle = false;
		for(l=1; l < point_counter; l++){
			nextq = next_q(next_p,0);

			if (new_edge(points[p], points[l],nextq) == 1){
				if(!inside_triangle(points[l],points[p])){
					graph[p][l] = distance_p(points[p], points[l]);
				}else{
					l = point_counter + 1;
					is_inside_a_triangle = true;
				}
			}else
				graph[p][l] = 0;
		}
		if(target_ and !is_inside_a_triangle){
			nextq = next_q(next_p,0);
			if (new_edge(points[SOURCE], points[TARGET],nextq) == 1){
					graph[SOURCE][TARGET] = distance_p(points[SOURCE], points[TARGET]);
					graph[TARGET][SOURCE] = 0;
			}
			else{
				graph[SOURCE][TARGET] = 0;
				graph[TARGET][SOURCE] = 0;
			}
		}

		return is_inside_a_triangle;

	}


#define NOTEDGE 0
#define FINISHED 1
#define BOUNDARY 2
#define NOT_REACHED 0



/****** HEAP code - Book: Advance Data Structures - Peter Brass *******/


typedef double key_t_;

typedef struct { key_t_ key; int node_id; int predecessor; }object_t;

typedef struct { key_t_ key; object_t *object; }heap_el_t;

typedef struct { int max_size; int current_size; heap_el_t *heap; } heap_t;

	heap_t *create_heap(int size){
		heap_t *hp;
		hp=(heap_t *) malloc( sizeof(heap_t) );
		hp->heap = (heap_el_t *) malloc( size * sizeof(heap_el_t) );
		hp->max_size = size;
		hp->current_size = 0;
		return( hp );
	}

	int heap_empty(heap_t *hp){ return( hp->current_size == 0 ); }

	heap_el_t *find_min(heap_t *hp){ return( hp->heap ); }

	int insert( key_t_ new_key, object_t *new_object, heap_t *hp){
		if ( hp->current_size < hp->max_size ){
			int gap;
			gap = hp->current_size++;
			while(gap > 0 && new_key < (hp->heap[(gap-1)/2]).key ){
				(hp->heap[gap]).key = (hp->heap[(gap-1)/2]).key;
				(hp->heap[gap]).object = (hp->heap[(gap-1)/2]).object;
				gap = (gap-1)/2;
			}
			(hp->heap[gap]).key = new_key;
			(hp->heap[gap]).object = new_object;
			return( 0 ); /* insert successful */
		}
		else
			return( -1 ); /* Heap overflow */
	}

object_t *delete_min(heap_t *hp){
	object_t *del_obj;
	int reached_top = 0;
	int gap, newgap, last;
	if( hp->current_size == 0 )
		return( NULL );
	/*failed: delete from empty heap */
	del_obj = (hp->heap[0]).object;
	gap = 0;
	while( ! reached_top ){
		if( 2*gap + 2 < hp->current_size ){
			if( (hp->heap[2*gap+1]).key < (hp->heap[2*gap+2]).key)
				newgap = 2*gap + 1;
			else
				newgap = 2*gap + 2;
			(hp->heap[gap]).key = (hp->heap[newgap]).key;
			(hp->heap[gap]).object = (hp->heap[newgap]).object;
			 gap = newgap;
		}
		else if ( 2*gap + 2 == hp->current_size ){
			newgap = 2*gap + 1;
			(hp->heap[gap]).key = (hp->heap[newgap]).key;
			(hp->heap[gap]).object = (hp->heap[newgap]).object;
			hp->current_size -= 1;
			return(del_obj);
			/* finished, came out exactly on last element */
		}
		else
			reached_top = 1;
	}
	/* propagated gap to the top, now move gap down again to insert
	 *  last object in the right place
	 *  */
	last = --hp->current_size;
	while(gap > 0 && (hp->heap[last]).key < (hp->heap[(gap-1)/2]).key )
	{
		(hp->heap[gap]).key = (hp->heap[(gap-1)/2]).key;
		(hp->heap[gap]).object = (hp->heap[(gap-1)/2]).object;
		gap = (gap-1)/2;
	}
	(hp->heap[gap]).key = (hp->heap[last]).key;
	(hp->heap[gap]).object = (hp->heap[last]).object;
	/* filled gap by moving last element in it*/
	return( del_obj );
}

void remove_heap(heap_t *hp){ free( hp->heap ); free( hp );
    
}


/*************************************************************************************************************/
/*                 Dijkstras algorithm with heaps  FIND THE SHORTEST PATH                      --------------*/
/*************************************************************************************************************/



/* Updates the boundary of the finished nodes in DIjkstra's algorithm
 * If the distance to a node is the same from two different nodes
 * the program keeps the old path.*/
bool update_boundary(object_t object, object_t *new_object, heap_t *hp, int status){

	object_t *temp_nodes1[NUMBER_POINTS] = {};
	object_t temp_nodes[NUMBER_POINTS]={};
	object_t *temp1 = (find_min(hp))->object;//address of object
	object_t temp = *(find_min(hp))->object;//value of object
	int counter = 0;int i = 0;
	bool found = false;
	bool update_predecessor = false;
	if(status == BOUNDARY){
		while(hp->current_size > 0  and !found){
			if(temp.node_id == object.node_id){
				if(temp.key > object.key){
					delete_min(hp);
					insert(object.key, new_object, hp);
					update_predecessor = true;
				}
				found = true;
			}else {
				temp_nodes1[counter] = temp1;
				temp_nodes[counter] = temp;
				delete_min(hp);
				temp1 = (find_min(hp))->object; //address of object
				temp = *(find_min(hp))->object; //value of object
				counter++;
			}
		}
		for (i=0;i<counter;i++){
			insert(temp_nodes[i].key, temp_nodes1[i], hp);
		}
	}else if (status == NOT_REACHED){
		insert(object.key, new_object, hp);
		update_predecessor = true;
	}
	return update_predecessor;
}

stack <point> shortest_path_dijkstras(){

	int i=0;
	int node_status[NUMBER_POINTS] = {};
	heap_t *heap1;
	stack <point> possible_path;
	heap1 = create_heap(NUMBER_POINTS) ;
	int predecessor[NUMBER_POINTS] = { };//index i is vertex  visited and value at that position is vertex i that it came from.
	bool update_predecessor = false;
	object_t new_object[NUMBER_POINTS*10] = {}; // It could be bigger
	//find neighbors of Source
	int temp_counter=0;
	for(i=1;i < point_counter +1; i++){
		if (graph[SOURCE][i] != 0){
			new_object[temp_counter].key = graph[SOURCE][i];
			new_object[temp_counter].node_id = i;
			new_object[temp_counter].predecessor = SOURCE;
			insert(graph[SOURCE][i], &(new_object[temp_counter]), heap1);
			node_status[i] = BOUNDARY;
			predecessor[i] =  -1;
			temp_counter ++;
		}
	}
	if (graph[SOURCE][TARGET] != 0){
		new_object[temp_counter].key = graph[SOURCE][TARGET];
		new_object[temp_counter].node_id = TARGET;
		new_object[temp_counter].predecessor = SOURCE;
		insert(graph[SOURCE][TARGET], &(new_object[temp_counter]), heap1);
		node_status[TARGET] = BOUNDARY;
		predecessor[TARGET] =  -1;
		temp_counter ++;
	}
	node_status[SOURCE]= FINISHED;

	double var; //double
	int temp_id;
	double temp_key;
	while(heap1->current_size > 0 and !(node_status[TARGET] == FINISHED)){
		object_t min_node = *(find_min(heap1))->object;
		temp_id = min_node.node_id;
		temp_key = min_node.key;
		delete_min(heap1);
		for(i=1;i<point_counter + 1;i++){
			if (((graph[temp_id][i] != 0) ||(graph[i][temp_id] != 0) ) and (node_status[i]!= FINISHED)){
				var = graph[temp_id][i] != 0? graph[temp_id][i] : graph[i][temp_id] ;
				new_object[temp_counter].key = var + temp_key;
				new_object[temp_counter].node_id = i;
				new_object[temp_counter].predecessor = temp_id;
				update_predecessor = update_boundary(new_object[temp_counter], &(new_object[temp_counter]),heap1,node_status[i]);
				node_status[i] = BOUNDARY;
				if(update_predecessor)
						predecessor[i] = temp_id;
				temp_counter ++;
			}
		}
		if (((graph[temp_id][TARGET] != 0) ||(graph[TARGET][temp_id] != 0) ) and (node_status[TARGET]!= FINISHED)){
			var = graph[temp_id][TARGET] != 0? graph[temp_id][TARGET] : graph[TARGET][temp_id] ;
			new_object[temp_counter].key = var + temp_key;
			new_object[temp_counter].node_id = TARGET;
			new_object[temp_counter].predecessor = temp_id;
			update_predecessor = update_boundary(new_object[temp_counter], &(new_object[temp_counter]),heap1,node_status[TARGET]);
			node_status[TARGET] = BOUNDARY;
			if(update_predecessor)
				predecessor[TARGET] = temp_id;
			temp_counter ++;
		}
		node_status[temp_id]= FINISHED;
	}

	int counter = TARGET;
	  remove_heap(heap1);
	  if(node_status[SOURCE] == FINISHED and node_status[TARGET] == FINISHED){
		  possible_path.push(points[TARGET]); //Puts in path the Target
		  	while (counter != SOURCE){
		  		if(predecessor[counter] == -1){
		  			possible_path.push(points[SOURCE]);
		  			counter = predecessor[SOURCE];
		  		}else{
		  			possible_path.push(points[predecessor[counter]]);
		  			counter = predecessor[counter];
		  		}
		  	}
		  shortest_path_found = true;
	  }
	  else{
		  while(!possible_path.empty()){
			  possible_path.pop();}
		  	  shortest_path_found = false;
		  	  printf("There is no path from Source to Target\n");
	  }
	return possible_path;
}

void delete_old_path(Display* display_ptr, Drawable win, GC gc_grey, GC gc_delete){
	point temp;
	if(!shortest_path.empty()){
		temp = shortest_path.top();
		shortest_path.pop();
		while(!shortest_path.empty()){
			if(temp.id == shortest_path.top().id)
				XDrawLine(display_ptr, win, gc_grey, temp.x, temp.y, shortest_path.top().x, shortest_path.top().y);
			else
				XDrawLine(display_ptr, win, gc_delete, temp.x, temp.y, shortest_path.top().x, shortest_path.top().y);

			temp = shortest_path.top();
			shortest_path.pop();
		}
}
	int i=0;
	for(i=0;i<NUMBER_POINTS;i++){
		graph[SOURCE][i] = 0;
		graph[TARGET][i] = 0;
	}

}

void draw_path(Display* display_ptr, Drawable win, GC gc_red, stack <point> path){
	point temp;
	if(!path.empty()){
		temp = path.top();
		path.pop();
		while(!path.empty()){
			XDrawLine(display_ptr, win, gc_red, temp.x, temp.y, path.top().x, path.top().y);
			temp = path.top();
			path.pop();
		}
}
}

/*************************************************************************************************************/
/*                 Reading Points From File                                                    --------------*/
/*************************************************************************************************************/


	void points_from_file(FILE *t){
		int px, py, qx, qy, rx,ry;
		int min_x, min_y;
		min_x = 0;

		while(!feof(t)){
			fscanf(t, "T (%d,%d) (%d,%d) (%d,%d)\n", &px, &py, &qx, &qy, &rx, &ry);
			if(min_x == 0){
				win_x = min(min(px,qx), rx); win_y = min(min(py,qy), ry);
				win_width = max(max(px,qx), rx); win_height = max(max(py,qy), ry);
				min_x=1;
			}else{
				win_x = min(win_x, min(min(px,qx), rx));
				win_y = min(win_y, min(min(py,qy), ry));
				win_width = max((int)win_width, max(max(px,qx), rx));
				win_height = max((int)win_height ,max(max(py,qy), ry));
			}

			points[count_points].x = px;
			points[count_points].y = py;
			points[count_points+1].x = qx;
			points[count_points+1].y = qy;
			points[count_points+2].x = rx;
			points[count_points+2].y = ry;

			 points[count_points].id = (count_points+2)/3;
			 points[count_points+1].id = (count_points+2)/3;
			 points[count_points+2].id = (count_points+2)/3;

			triangles_graph[count_points][count_points+1] = 1;
			triangles_graph[count_points][count_points+2] = 1;
			triangles_graph[count_points+1][count_points+2] = 1;

			count_points = count_points + 3;
			point_counter = point_counter + 3;

		}fclose(t);
		min_x = win_x - ((win_width - win_x )* 0.1);
		min_y = win_y - ((win_height - win_y)* 0.1);

		win_width = (win_width - win_x )* 1.1;
		win_height = (win_height - win_y)* 1.1;
		win_x = min_x;
		win_y = min_y;
	}


int main(int argc, char *argv[])
{
	//clock_t tSart = clock();
	if(argc > 1){
		fp = fopen(argv[1], "r");
		points_from_file(fp);
	}else
		printf("File was not given.\n");

	stack <point> temp_path ;
	int i = 0;

/*-----------------------WINDOW DISPLAY: starts ----------------------------*/

  /* opening display: basic connection to X Server */
  if( (display_ptr = XOpenDisplay(display_name)) == NULL )
    {
	  printf("Could not open display. \n"); exit(-1);
    }

  printf("Connected to X server  %s\n", XDisplayName(display_name) );
  screen_num = DefaultScreen( display_ptr );
  screen_ptr = DefaultScreenOfDisplay( display_ptr );
  color_map  = XDefaultColormap( display_ptr, screen_num );
  display_width  = DisplayWidth( display_ptr, screen_num );
  display_height = DisplayHeight( display_ptr, screen_num );

  printf("Width %d, Height %d, Screen Number %d\n",
		  display_width, display_height, screen_num);

  /* creating the window */
  border_width = 10;
  if(argc <2){
  	  win_x = 100; win_y = 100; win_width = display_width/1.7; win_height = (int) (win_width /1.1);
  } /*rectangular window*/

  win= XCreateSimpleWindow( display_ptr, RootWindow( display_ptr, screen_num),
		  win_x, win_y, win_width, win_height, border_width,
		  BlackPixel(display_ptr, screen_num),
		  WhitePixel(display_ptr, screen_num) );

  /* now try to put it on screen, this needs cooperation of window manager */
  size_hints = XAllocSizeHints();
  wm_hints = XAllocWMHints();
  class_hints = XAllocClassHint();
  if( size_hints == NULL || wm_hints == NULL || class_hints == NULL )
  { printf("Error allocating memory for hints. \n"); exit(-1);}

	  size_hints -> flags = PPosition | PSize | PMinSize  ;
	  size_hints -> min_width = 60;
	  size_hints -> min_height = 60;

	  XStringListToTextProperty( &win_name_string,1,&win_name);
	  XStringListToTextProperty( &icon_name_string,1,&icon_name);

  wm_hints -> flags = StateHint | InputHint ;
  wm_hints -> initial_state = NormalState;
  wm_hints -> input = False;

  class_hints -> res_name = "x_use_example";
  class_hints -> res_class = "examples";

  XSetWMProperties( display_ptr, win, &win_name, &icon_name, argv, argc,
                    size_hints, wm_hints, class_hints );

  /* what events do we want to receive */
  XSelectInput( display_ptr, win,
            ExposureMask | StructureNotifyMask | ButtonPressMask );

  /* finally: put window on screen */
  XMapWindow( display_ptr, win );

  XFlush(display_ptr);

  /* create graphics context, so that we may draw in this window */
  gc = XCreateGC( display_ptr, win, valuemask, &gc_values);
  XSetForeground( display_ptr, gc, BlackPixel( display_ptr, screen_num ) );
  XSetLineAttributes( display_ptr, gc, 4, LineSolid, CapRound, JoinRound);

  /* and three other graphics contexts, to draw in yellow and red and grey white=WhitePixel(dis, screen);*/
  gc_delete = XCreateGC( display_ptr, win, valuemask, &gc_white_values);
  XSetLineAttributes(display_ptr, gc_delete, 1, LineSolid,CapRound, JoinRound);
  if( XAllocNamedColor( display_ptr, color_map, "white",
			&tmp_color1, &tmp_color2 ) == 0 )
    {printf("failed to get color white\n"); exit(-1);}
  else
    XSetForeground( display_ptr, gc_delete, WhitePixel( display_ptr, screen_num ) );

  /* other graphics contexts red*/
  gc_red = XCreateGC( display_ptr, win, valuemask, &gc_red_values);
  XSetLineAttributes( display_ptr, gc_red, 1, LineSolid, CapRound, JoinRound);
  if( XAllocNamedColor( display_ptr, color_map, "red",
			&tmp_color1, &tmp_color2 ) == 0 )
    {printf("failed to get color red\n"); exit(-1);}
  else
    XSetForeground( display_ptr, gc_red, tmp_color1.pixel );

  /*other graphics contexts grey*/
  gc_grey = XCreateGC( display_ptr, win, valuemask, &gc_grey_values);
  XSetLineAttributes( display_ptr, gc_grey, 2, LineSolid, CapRound, JoinRound);
  if( XAllocNamedColor( display_ptr, color_map, "light grey",
			&tmp_color1, &tmp_color2 ) == 0 )
    {printf("failed to get color grey\n"); exit(-1);}
  else
    XSetForeground( display_ptr, gc_grey, tmp_color1.pixel );

  /* and now it starts: the event loop */
  while(1)
    {

	  XNextEvent( display_ptr, &report );
      switch( report.type )
      {
      	  case Expose:

      		  if (count_points > 1){
      		  for (i = 1; i < point_counter -1 ; i = i + 3){
      			  XDrawLine(display_ptr, win, gc_grey, points[i].x, points[i].y,
      					  points[i+1].x, points[i+1].y );
      			  XDrawLine(display_ptr, win, gc_grey, points[i].x, points[i].y,
      					  points[i+2].x, points[i+2].y );
      			  XDrawLine( display_ptr, win, gc_grey, points[i+1].x, points[i+1].y,
      					  points[i+2].x, points[i+2].y );
      		  }
   			  /*redraw the shortest path*/
      		  draw_path(display_ptr,win,gc_red,shortest_path);
      		  }
      		  break;

      	  case ConfigureNotify:
      		  /* This event happens when the user changes the size of the window*/
      		  win_width = report.xconfigure.width;
      		  win_height = report.xconfigure.height;
      		  break;

      	  case ButtonPress:{
      		  if (report.xbutton.button == Button1 )  //left click
      		  {
      			  if(count_points < NUMBER_POINTS-1){
      				  points[count_points].x = report.xbutton.x;
      				  points[count_points].y = report.xbutton.y;
      				  XDrawPoint(display_ptr,win,gc_red,points[count_points].x,points[count_points].y);
      				  points_in_triangule++;
      				  if(points_in_triangule == 3){
      					  triangles_graph[count_points-2][count_points-1] = 1;
      					  triangles_graph[count_points-2][count_points] = 1;
      					  triangles_graph[count_points-1][count_points] = 1;
      					  points[count_points].id = count_points/3;
      					  points[count_points-1].id = count_points/3;
      					  points[count_points-2].id = count_points/3;

      					  // Draw the triangle;
      					  XDrawLine(display_ptr, win, gc_grey, points[count_points].x, points[count_points].y,
      							  points[count_points-1].x, points[count_points-1].y);
      					  XDrawLine(display_ptr, win, gc_grey, points[count_points].x, points[count_points].y,
      							  points[count_points-2].x, points[count_points-2].y);
      					  XDrawLine(display_ptr, win, gc_grey, points[count_points-2].x, points[count_points-2].y,
      							  points[count_points-1].x, points[count_points-1].y);
      					  points_in_triangule = 0;
      					point_counter = point_counter + 3;
      				  }
      				  count_points++;
      			  }else if (s_t < 2 and counter_right_click == 1) {
      				  if (s_t == 0){
      					delete_old_path(display_ptr,win,gc_grey, gc_delete);
      					  points[SOURCE].x =report.xbutton.x;
      					  points[SOURCE].y =report.xbutton.y;
      					if(!visible_points(SOURCE, false)){
      						XDrawPoint(display_ptr,win,gc_grey,points[SOURCE].x,points[SOURCE].y);
      						s_t++;
      					}else
      						printf("The Source Point should be outside a triangle\n");

      				  }else{
      					  points[TARGET].x =report.xbutton.x;
      					  points[TARGET].y =report.xbutton.y;

      					  if(!visible_points(TARGET, true)){
      						  XDrawPoint(display_ptr,win,gc_grey,points[TARGET].x,points[TARGET].y);
      						  points[TARGET].id = TARGET;
      						  shortest_path = shortest_path_dijkstras();
      						  draw_path(display_ptr,win,gc_red,shortest_path);
     						  s_t = 0;
      					  }else
      						  printf("The Target Point should be outside a triangle\n");
      				  }
      			  }else
      				printf("You have exceeded the maximum number of points\n");
      		  }
      		  else { //Right click
      			  if(counter_right_click == 0){
      				  while(points_in_triangule >= 0 and count_points > 0){
      					  points[count_points].x = 0; // count_points = 0 is reserved for source.
      					  points[count_points].y = 0;
      					  points_in_triangule--;
      					  count_points--;
      				  }
      				build_graph_triangles();
      				counter_right_click++;
      				count_points = NUMBER_POINTS;
      			  }else{
      				s_t = 2;
      				XFreeGC(display_ptr, gc);
      				XFreeGC(display_ptr, gc_grey);
      				XFreeGC(display_ptr, gc_delete);
      				XFreeGC(display_ptr, gc_red);
      				XDestroyWindow(display_ptr,win);
      				XCloseDisplay(display_ptr);
      				//printf("Time taken: %.2fs\n", (double)(clock() - tSart)/CLOCKS_PER_SEC);
      			  }
      		  }
      	  }
      	  break;
      	  default:
      		  /* this is a catch-all for other events; it does not do anything.
             One could look at the report type to see what the event was */
      		  break;
      }
    }
  exit(0);
}
