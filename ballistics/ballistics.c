#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Physical constants in imperial units
#define GRAVITY_FPS2 32.174      // ft/s^2
#define YARDS_TO_FT 3.0
#define FT_TO_INCHES 12.0
#define MAX_RANGE_YARDS 1000     // Maximum range for trajectory table

// Standard atmospheric conditions
#define STD_AIR_DENSITY_LBFT3 0.0764742  // lb/ft^3 at sea level, 59°F
#define SPEED_OF_SOUND_FPS 1116.45       // fps at sea level, 59°F

// G1 standard projectile reference area (1 inch diameter = 1 lb, 1 inch)
// For G1: reference sectional density = 1 lb/in^2
#define G1_REF_SECTIONAL_DENSITY 1.0

// G1 drag coefficient table (Cd vs Mach number)
// Based on Ingalls/Mayevski G1 standard projectile
// Mach, Cd pairs - we'll interpolate between these
static const double G1_DRAG_TABLE[][2] = {
    {0.20, 0.2344}, {0.25,  0.2278}, {0.30, 0.2214}, {0.35, 0.2155},
    {0.00, 0.2629}, {0.05,  0.2558}, {0.10, 0.2487}, {0.15, 0.2413},
    {0.40, 0.2104}, {0.45,  0.2061}, {0.50, 0.2032}, {0.55, 0.2020},
    {0.60, 0.2034}, {0.65,  0.2165}, {0.70, 0.2230}, {0.75, 0.2313},
    {0.80, 0.2417}, {0.85,  0.2546}, {0.90, 0.2706}, {0.925, 0.2859},
    {0.95, 0.3052}, {0.975, 0.3310}, {1.00, 0.3611}, {1.025, 0.3966},
    {1.05, 0.4323}, {1.075, 0.4641}, {1.10, 0.4895}, {1.125, 0.5085},
    {1.15, 0.5212}, {1.175, 0.5293}, {1.20, 0.5340}, {1.225, 0.5367},
    {1.25, 0.5385}, {1.30,  0.5399}, {1.35, 0.5399}, {1.40, 0.5388},
    {1.45, 0.5368}, {1.50,  0.5343}, {1.55, 0.5315}, {1.60, 0.5284},
    {1.65, 0.5253}, {1.70,  0.5221}, {1.75, 0.5190}, {1.80, 0.5159},
    {1.85, 0.5128}, {1.90,  0.5098}, {1.95, 0.5068}, {2.00, 0.5039},
    {2.05, 0.5011}, {2.10,  0.4983}, {2.15, 0.4956}, {2.20, 0.4929},
    {2.25, 0.4903}, {2.30,  0.4878}, {2.35, 0.4854}, {2.40, 0.4830},
    {2.45, 0.4807}, {2.50,  0.4785}, {2.55, 0.4764}, {2.60, 0.4743},
    {2.65, 0.4723}, {2.70,  0.4704}, {2.75, 0.4685}, {2.80, 0.4667},
    {2.85, 0.4650}, {2.90,  0.4633}, {2.95, 0.4617}, {3.00, 0.4601},
    {3.10, 0.4570}, {3.20,  0.4541}, {3.30, 0.4513}, {3.40, 0.4486},
    {3.50, 0.4461}, {3.60,  0.4437}, {3.70, 0.4413}, {3.80, 0.4391},
    {3.90, 0.4369}, {4.00,  0.4349}, {4.20, 0.4310}, {4.40, 0.4275},
    {4.60, 0.4242}, {4.80,  0.4211}, {5.00, 0.4182}
};
#define G1_TABLE_SIZE (sizeof(G1_DRAG_TABLE) / sizeof(G1_DRAG_TABLE[0]))

// G7 drag coefficient table (Cd vs Mach number)
// Based on boat-tail, secant ogive projectile (modern match bullets)
static const double G7_DRAG_TABLE[][2] = {
    {0.00, 0.1198}, {0.05,  0.1197}, {0.10, 0.1196}, {0.15, 0.1194},
    {0.20, 0.1193}, {0.25,  0.1194}, {0.30, 0.1194}, {0.35, 0.1194},
    {0.40, 0.1193}, {0.45,  0.1193}, {0.50, 0.1194}, {0.55, 0.1193},
    {0.60, 0.1194}, {0.65,  0.1197}, {0.70, 0.1202}, {0.725, 0.1207},
    {0.75, 0.1215}, {0.775, 0.1226}, {0.80, 0.1242}, {0.825, 0.1266},
    {0.85, 0.1306}, {0.875, 0.1368}, {0.90, 0.1464}, {0.925, 0.1660},
    {0.95, 0.2054}, {0.975, 0.2993}, {1.00, 0.3803}, {1.025, 0.4015},
    {1.05, 0.4043}, {1.075, 0.4034}, {1.10, 0.4014}, {1.125, 0.3987},
    {1.15, 0.3955}, {1.175, 0.3921}, {1.20, 0.3887}, {1.225, 0.3853},
    {1.25, 0.3819}, {1.30,  0.3752}, {1.35, 0.3687}, {1.40, 0.3625},
    {1.45, 0.3566}, {1.50,  0.3510}, {1.55, 0.3457}, {1.60, 0.3407},
    {1.65, 0.3361}, {1.70,  0.3317}, {1.75, 0.3275}, {1.80, 0.3236},
    {1.85, 0.3199}, {1.90,  0.3165}, {1.95, 0.3132}, {2.00, 0.3101},
    {2.05, 0.3072}, {2.10,  0.3045}, {2.15, 0.3019}, {2.20, 0.2995},
    {2.25, 0.2972}, {2.30,  0.2950}, {2.35, 0.2930}, {2.40, 0.2911},
    {2.45, 0.2893}, {2.50,  0.2876}, {2.55, 0.2860}, {2.60, 0.2845},
    {2.65, 0.2830}, {2.70,  0.2817}, {2.75, 0.2804}, {2.80, 0.2792},
    {2.85, 0.2780}, {2.90,  0.2769}, {2.95, 0.2759}, {3.00, 0.2749},
    {3.10, 0.2732}, {3.20,  0.2716}, {3.30, 0.2702}, {3.40, 0.2689},
    {3.50, 0.2678}, {3.60,  0.2667}, {3.70, 0.2658}, {3.80, 0.2650},
    {3.90, 0.2642}, {4.00,  0.2635}, {4.20, 0.2622}, {4.40, 0.2612},
    {4.60, 0.2603}, {4.80,  0.2595}, {5.00, 0.2589}
};
#define G7_TABLE_SIZE (sizeof(G7_DRAG_TABLE) / sizeof(G7_DRAG_TABLE[0]))

// Drag model enum
typedef enum {
    DRAG_G1 = 1,
    DRAG_G7 = 7
} DragModel;

// Projectile parameters (all imperial units)
typedef struct {
    double muzzle_velocity_fps;   // Muzzle velocity in feet per second
    double bullet_weight_gr;      // Bullet weight in grains
    double ballistic_coef;        // Ballistic coefficient (G1 or G7)
    double zero_range_yards;      // Zero range in yards
    double sight_height_inches;   // Sight height above bore in inches
    char name[64];                // Name/description of the load
    DragModel drag_model;         // G1 or G7
} ProjectileInput;

// Trajectory state at a point
typedef struct {
    double range_yards;           // Horizontal distance in yards
    double drop_inches;           // Drop from line of sight in inches
    double velocity_fps;          // Current velocity in fps
    double time_sec;              // Time of flight in seconds
    double energy_ftlbs;          // Kinetic energy in ft-lbs
} TrajectoryPoint;

// Calculate kinetic energy in ft-lbs
// KE = (1/2) * m * v^2
// With grains and fps: KE = (bullet_weight_gr * velocity^2) / 450437
double calc_energy_ftlbs(double weight_gr, double velocity_fps) {
    return (weight_gr * velocity_fps * velocity_fps) / 450437.0;
}

// Interpolate G1 drag coefficient from table based on Mach number
double get_g1_cd(double mach) {
    // Clamp to table range
    if (mach <= G1_DRAG_TABLE[0][0]) {
        return G1_DRAG_TABLE[0][1];
    }
    if (mach >= G1_DRAG_TABLE[G1_TABLE_SIZE - 1][0]) {
        return G1_DRAG_TABLE[G1_TABLE_SIZE - 1][1];
    }
    
    // Linear interpolation
    for (int i = 0; i < G1_TABLE_SIZE - 1; i++) {
        if (mach >= G1_DRAG_TABLE[i][0] && mach < G1_DRAG_TABLE[i + 1][0]) {
            double m0 = G1_DRAG_TABLE[i][0];
            double m1 = G1_DRAG_TABLE[i + 1][0];
            double cd0 = G1_DRAG_TABLE[i][1];
            double cd1 = G1_DRAG_TABLE[i + 1][1];
            double t = (mach - m0) / (m1 - m0);
            return cd0 + t * (cd1 - cd0);
        }
    }
    return G1_DRAG_TABLE[G1_TABLE_SIZE - 1][1];
}

// Interpolate G7 drag coefficient from table based on Mach number
double get_g7_cd(double mach) {
    // Clamp to table range
    if (mach <= G7_DRAG_TABLE[0][0]) {
        return G7_DRAG_TABLE[0][1];
    }
    if (mach >= G7_DRAG_TABLE[G7_TABLE_SIZE - 1][0]) {
        return G7_DRAG_TABLE[G7_TABLE_SIZE - 1][1];
    }
    
    // Linear interpolation
    for (int i = 0; i < G7_TABLE_SIZE - 1; i++) {
        if (mach >= G7_DRAG_TABLE[i][0] && mach < G7_DRAG_TABLE[i + 1][0]) {
            double m0 = G7_DRAG_TABLE[i][0];
            double m1 = G7_DRAG_TABLE[i + 1][0];
            double cd0 = G7_DRAG_TABLE[i][1];
            double cd1 = G7_DRAG_TABLE[i + 1][1];
            double t = (mach - m0) / (m1 - m0);
            return cd0 + t * (cd1 - cd0);
        }
    }
    return G7_DRAG_TABLE[G7_TABLE_SIZE - 1][1];
}

// Get drag coefficient based on model type
double get_cd(double mach, DragModel model) {
    if (model == DRAG_G7) {
        return get_g7_cd(mach);
    }
    return get_g1_cd(mach);  // Default to G1
}

// Calculate drag deceleration (ft/s^2)
// Using the relation: a_drag = (Cd * rho * A * v^2) / (2 * m)
// With BC: a_drag = (Cd_g1 / BC) * (rho / rho_std) * v^2 * constant
// Simplified form for G1: retardation = Cd_g1(mach) * v^2 / (BC * constant)
double calc_drag_decel(double velocity_fps, double bc, DragModel model) {
    double mach = velocity_fps / SPEED_OF_SOUND_FPS;
    double cd = get_cd(mach, model);
    
    // The drag function returns deceleration per unit BC
    // Deceleration (ft/s^2) = Cd * v^2 / (BC * k)
    // where k is a constant that makes the units work out
    // For G1 with standard atmosphere: k ≈ 2367
    // For G7: same constant works because BC is scaled appropriately
    double k = 2367.0;
    
    double decel = cd * velocity_fps * velocity_fps / (bc * k);
    return decel;
}

// Trajectory state for numerical integration
typedef struct {
    double x;    // Horizontal position (ft)
    double y;    // Vertical position (ft)
    double vx;   // Horizontal velocity (ft/s)
    double vy;   // Vertical velocity (ft/s)
    double t;    // Time (s)
} State;

// Calculate derivatives for RK4 integration
void calc_derivatives(State *s, double bc, DragModel model, double *dx, double *dy, double *dvx, double *dvy) {
    double v = sqrt(s->vx * s->vx + s->vy * s->vy);
    
    // Drag deceleration magnitude
    double drag_decel = calc_drag_decel(v, bc, model);
    
    // Drag acts opposite to velocity direction
    double drag_x = (v > 0) ? -drag_decel * (s->vx / v) : 0;
    double drag_y = (v > 0) ? -drag_decel * (s->vy / v) : 0;
    
    *dx = s->vx;
    *dy = s->vy;
    *dvx = drag_x;
    *dvy = -GRAVITY_FPS2 + drag_y;  // Gravity + vertical drag component
}

// RK4 integration step
void rk4_step(State *s, double bc, DragModel model, double dt) {
    double dx1, dy1, dvx1, dvy1;
    double dx2, dy2, dvx2, dvy2;
    double dx3, dy3, dvx3, dvy3;
    double dx4, dy4, dvx4, dvy4;
    
    // k1
    calc_derivatives(s, bc, model, &dx1, &dy1, &dvx1, &dvy1);
    
    // k2
    State s2 = {
        s->x + dx1 * dt / 2, s->y + dy1 * dt / 2,
        s->vx + dvx1 * dt / 2, s->vy + dvy1 * dt / 2, s->t
    };
    calc_derivatives(&s2, bc, model, &dx2, &dy2, &dvx2, &dvy2);
    
    // k3
    State s3 = {
        s->x + dx2 * dt / 2, s->y + dy2 * dt / 2,
        s->vx + dvx2 * dt / 2, s->vy + dvy2 * dt / 2, s->t
    };
    calc_derivatives(&s3, bc, model, &dx3, &dy3, &dvx3, &dvy3);
    
    // k4
    State s4 = {
        s->x + dx3 * dt, s->y + dy3 * dt,
        s->vx + dvx3 * dt, s->vy + dvy3 * dt, s->t
    };
    calc_derivatives(&s4, bc, model, &dx4, &dy4, &dvx4, &dvy4);
    
    // Update state
    s->x += dt * (dx1 + 2*dx2 + 2*dx3 + dx4) / 6;
    s->y += dt * (dy1 + 2*dy2 + 2*dy3 + dy4) / 6;
    s->vx += dt * (dvx1 + 2*dvx2 + 2*dvx3 + dvx4) / 6;
    s->vy += dt * (dvy1 + 2*dvy2 + 2*dvy3 + dvy4) / 6;
    s->t += dt;
}

// Calculate trajectory with drag, returning height at specified range
// Used for zero angle iteration
double calc_height_at_range_with_drag(double launch_angle, double v0, double bc, 
                                       DragModel model, double target_range_ft) {
    State s = {0, 0, v0 * cos(launch_angle), v0 * sin(launch_angle), 0};
    double dt = 0.0001;  // 0.1 ms time step for accuracy
    
    while (s.x < target_range_ft && s.t < 10.0) {
        rk4_step(&s, bc, model, dt);
    }
    
    return s.y;
}

// Find zero angle using bisection method
// The bullet must cross the line of sight at zero_range
double find_zero_angle_with_drag(double v0, double bc, DragModel model,
                                  double zero_range_yards, double sight_height_inches) {
    double zero_range_ft = zero_range_yards * YARDS_TO_FT;
    double sight_height_ft = sight_height_inches / FT_TO_INCHES;
    
    // At zero range, bullet path should be at same height as where LOS points
    // LOS starts at sight_height and angles down to hit target
    // For simplicity: find angle where bullet is at sight_height at zero_range
    // (this means bullet crosses from below LOS to above and back to LOS level)
    // Actually: bullet should be at height 0 at zero_range if we measure from aim point
    // Let's use: bullet height at zero_range should equal sight_height
    // (bullet rises to meet the line of sight)
    
    // Bisection search
    double angle_low = 0.0;
    double angle_high = 0.1;  // ~5.7 degrees max
    double target_height = sight_height_ft;  // Bullet must reach sight height at zero
    
    for (int i = 0; i < 50; i++) {
        double angle_mid = (angle_low + angle_high) / 2;
        double height = calc_height_at_range_with_drag(angle_mid, v0, bc, model, zero_range_ft);
        
        if (height < target_height) {
            angle_low = angle_mid;
        } else {
            angle_high = angle_mid;
        }
        
        if (fabs(height - target_height) < 0.0001) {
            break;
        }
    }
    
    return (angle_low + angle_high) / 2;
}

// Calculate full trajectory with drag model
void calc_drag_trajectory(ProjectileInput *input, TrajectoryPoint *points, 
                          int *num_points, int max_points) {
    double v0 = input->muzzle_velocity_fps;
    double bc = input->ballistic_coef;
    double zero_range = input->zero_range_yards;
    double sight_height = input->sight_height_inches;
    DragModel model = input->drag_model;
    
    // Find launch angle for zero (iterative with drag)
    double theta = find_zero_angle_with_drag(v0, bc, model, zero_range, sight_height);
    
    // Initialize state
    State s = {0, 0, v0 * cos(theta), v0 * sin(theta), 0};
    double dt = 0.0001;  // 0.1 ms time step
    
    // Calculate LOS angle (from scope to zero point)
    double zero_range_ft = zero_range * YARDS_TO_FT;
    double sight_height_ft = sight_height / FT_TO_INCHES;
    double los_angle = atan2(sight_height_ft - sight_height_ft, zero_range_ft);
    // Actually: LOS from (0, sight_height) to (zero_range, sight_height) is horizontal
    // No wait - at zero, bullet is AT sight height level. LOS aims at where bullet hits.
    // The bullet height at zero range = sight_height, and that's where we're aiming.
    // So LOS is horizontal if we consider the aim point is at bullet height at zero range.
    // For drop calculation: measure vertical distance from LOS to bullet
    // LOS at range x: y_los = sight_height_ft (horizontal line to zero point)
    // After zero: LOS continues straight
    
    // Actually for a proper POI (point of impact):
    // At zero range, bullet impacts where we aim. The scope is sight_height above bore.
    // We aim at a point, and the bullet arcs up then down to hit it at zero range.
    // Beyond zero, bullet is below aim point.
    //
    // For drop output: drop = 0 at zero range
    // Before zero: bullet may be slightly high (negative drop convention varies)
    // After zero: bullet is low (positive drop in most conventions, but we use negative)
    
    los_angle = atan2(sight_height_ft - sight_height_ft, zero_range_ft); // = 0 (horizontal)
    // Hmm, LOS should actually aim where bullet goes, which accounts for bullet rise
    // Let me simplify: 
    // - Drop is measured from line of sight
    // - LOS goes from scope to where bullet impacts at zero range
    // - At zero: drop = 0 by definition
    // - Bullet height at zero = sight_height_ft (from our angle calculation)
    // - LOS goes from (0, sight_height_ft) to (zero_range_ft, sight_height_ft) - horizontal!
    // No that's wrong too...
    
    // The aiming line (LOS) points from the scope, through the reticle, to infinity.
    // When zeroed at 100 yards, the bullet path crosses this line at 100 yards.
    // For our coordinate system: bore is at y=0, scope is at y=sight_height.
    // Bullet starts at (0,0), rises due to angle, then falls.
    // At zero_range, bullet is at some height h.
    // LOS goes from (0, sight_height) toward (zero_range, h) and beyond.
    // Since bullet crosses LOS at zero, h is on the LOS.
    // LOS slope = (h - sight_height) / zero_range
    // LOS at range x: y_los = sight_height + x * (h - sight_height) / zero_range
    //                       = sight_height * (1 - x/zero_range) + h * (x/zero_range)
    
    // First, calculate bullet height at zero range with drag
    double h_at_zero = calc_height_at_range_with_drag(theta, v0, bc, model, zero_range_ft);
    los_angle = atan2(h_at_zero - sight_height_ft, zero_range_ft);
    
    int idx = 0;
    int next_range_yards = 0;
    
    // Step through trajectory
    while (s.x / YARDS_TO_FT <= MAX_RANGE_YARDS && s.t < 10.0 && idx < max_points) {
        double current_range_yards = s.x / YARDS_TO_FT;
        
        // Record data at 50-yard increments
        if (current_range_yards >= next_range_yards) {
            double los_height_ft = sight_height_ft + s.x * tan(los_angle);
            double drop_ft = s.y - los_height_ft;
            double drop_inches = drop_ft * FT_TO_INCHES;
            
            double v = sqrt(s.vx * s.vx + s.vy * s.vy);
            
            points[idx].range_yards = next_range_yards;
            points[idx].drop_inches = drop_inches;
            points[idx].velocity_fps = v;
            points[idx].time_sec = s.t;
            points[idx].energy_ftlbs = calc_energy_ftlbs(input->bullet_weight_gr, v);
            
            idx++;
            next_range_yards += 50;
        }
        
        rk4_step(&s, bc, model, dt);
    }
    
    *num_points = idx;
}

// Calculate launch angle to hit zero at specified range (vacuum trajectory)
// The bullet starts below the sight line by sight_height inches.
// We need to find the angle that makes the bullet cross the line of sight at zero_range.
double calc_zero_angle_rad(double velocity_fps, double zero_range_yards, 
                            double sight_height_inches) {
    double range_ft = zero_range_yards * YARDS_TO_FT;
    double sight_height_ft = sight_height_inches / FT_TO_INCHES;
    double v_sq = velocity_fps * velocity_fps;
    
    // The line of sight is angled downward from scope to zero point.
    // At zero range, the bullet (starting sight_height below LOS) must reach LOS.
    // 
    // Bullet height at range R: h = R*tan(θ) - g*R²/(2*v²*cos²(θ))
    // For small angles: h ≈ R*θ - g*R²/(2*v²)
    // 
    // The LOS goes from (0, sight_height) to (zero_range, 0).
    // LOS slope = -sight_height / zero_range
    // LOS at range R: y_los = sight_height - (sight_height/zero_range)*R
    // 
    // At zero_range, bullet must be at y=0 (hits target at ground level relative to aim)
    // But bullet starts at y=0 (bore), while we measure from bore.
    // So at zero_range, bullet height from bore = 0 (LOS crosses bore)
    // Actually bullet must be at sight_height at range=0 perspective...
    //
    // Simpler: bullet needs to rise by sight_height over zero_range, then gravity pulls it down
    // h(R) = R*θ - g*R²/(2*v²) = 0 at R = zero_range when it crosses LOS
    // But bullet starts below LOS by sight_height...
    //
    // Let me reconsider from scratch:
    // - Bore is at height 0
    // - Scope/sight is at height +sight_height
    // - We aim at target at zero_range (LOS points at it)
    // - Bullet must cross the LOS at zero_range
    // - At zero_range, LOS is at some height above ground... 
    //
    // Actually for a proper zero: bullet path crosses LOS at zero distance.
    // The LOS has a slight downward angle from scope to target.
    // For small angle approximation:
    //   θ_bore = g*R/(2*v²) + sight_height/R
    
    double angle_rad = (GRAVITY_FPS2 * range_ft) / (2.0 * v_sq) + sight_height_ft / range_ft;
    
    return angle_rad;
}

// Calculate vacuum trajectory (no drag)
// This is a simple parabolic path
void calc_vacuum_trajectory(ProjectileInput *input, TrajectoryPoint *points, 
                            int *num_points, int max_points) {
    double v0 = input->muzzle_velocity_fps;
    double zero_range = input->zero_range_yards;
    double sight_height = input->sight_height_inches;
    
    // Calculate launch angle for zero
    double theta = calc_zero_angle_rad(v0, zero_range, sight_height);
    
    double v0x = v0 * cos(theta);  // Horizontal velocity (constant in vacuum)
    double v0y = v0 * sin(theta);  // Initial vertical velocity
    
    // Line of sight angle (from scope to zero point)
    // LOS goes from (0, sight_height) to (zero_range, bullet_height_at_zero)
    // bullet_height_at_zero = sight_height (for the LOS to intersect bullet path)
    // Wait, the bullet crosses LOS, so at zero_range, bullet is ON the LOS.
    // LOS starts at scope (height=sight_height above bore) and hits where we aim.
    // If zeroed at 100 yards, bullet impacts at aim point at 100 yards.
    // 
    // For drop calculation standard: drop is measured from LOS (where you're aiming)
    // Drop = 0 at zero range
    // Drop is negative when bullet is above LOS (between muzzle and zero)
    // Drop is positive when bullet is below LOS (past zero)
    
    // Calculate bullet height at zero range to establish LOS
    double zero_range_ft = zero_range * YARDS_TO_FT;
    double t_zero = zero_range_ft / v0x;
    double bullet_height_at_zero = v0y * t_zero - 0.5 * GRAVITY_FPS2 * t_zero * t_zero;
    
    // LOS angle from horizontal (scope to zero point)
    // LOS starts at sight_height, ends at bullet_height_at_zero at zero_range
    double los_angle = atan2(bullet_height_at_zero - sight_height / FT_TO_INCHES, zero_range_ft);
    
    int idx = 0;
    for (double range = 0; range <= MAX_RANGE_YARDS && idx < max_points; range += 100) {
        double range_ft = range * YARDS_TO_FT;
        
        // Time to reach this range
        double t = range_ft / v0x;
        
        // Bullet height above bore
        double bullet_height_ft = v0y * t - 0.5 * GRAVITY_FPS2 * t * t;
        
        // LOS height at this range
        // LOS: y = sight_height + range * tan(los_angle)
        double los_height_ft = (sight_height / FT_TO_INCHES) + range_ft * tan(los_angle);
        
        // Drop = bullet - LOS (negative = bullet below LOS = drop)
        double drop_ft = bullet_height_ft - los_height_ft;
        double drop_inches = drop_ft * FT_TO_INCHES;
        
        // Velocity magnitude
        double vy = v0y - GRAVITY_FPS2 * t;
        double v = sqrt(v0x * v0x + vy * vy);
        
        // Store results
        points[idx].range_yards = range;
        points[idx].drop_inches = drop_inches;
        points[idx].velocity_fps = v;
        points[idx].time_sec = t;
        points[idx].energy_ftlbs = calc_energy_ftlbs(input->bullet_weight_gr, v);
        
        idx++;
    }
    
    *num_points = idx;
}

// Parse input file
// Format:
//   name=.308 Win 168gr SMK
//   muzzle_velocity=2650
//   bullet_weight=168
//   ballistic_coef=0.223
//   drag_model=G7
//   zero_range=100
//   sight_height=1.5
int parse_input_file(const char *filename, ProjectileInput *input) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "Error: Cannot open input file '%s'\n", filename);
        return -1;
    }

    // Set defaults
    memset(input, 0, sizeof(ProjectileInput));
    strcpy(input->name, "Unknown");
    input->drag_model = DRAG_G1;  // Default to G1

    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n') continue;

        char key[64], value[128];
        if (sscanf(line, "%63[^=]=%127[^\n]", key, value) == 2) {
            // Trim whitespace
            char *k = key, *v = value;
            while (*k == ' ') k++;
            while (*v == ' ') v++;

            if (strcmp(k, "name") == 0) {
                strncpy(input->name, v, sizeof(input->name) - 1);
            } else if (strcmp(k, "muzzle_velocity") == 0) {
                input->muzzle_velocity_fps = atof(v);
            } else if (strcmp(k, "bullet_weight") == 0) {
                input->bullet_weight_gr = atof(v);
            } else if (strcmp(k, "ballistic_coef") == 0) {
                input->ballistic_coef = atof(v);
            } else if (strcmp(k, "drag_model") == 0) {
                if (strcmp(v, "G7") == 0 || strcmp(v, "g7") == 0) {
                    input->drag_model = DRAG_G7;
                } else {
                    input->drag_model = DRAG_G1;
                }
            } else if (strcmp(k, "zero_range") == 0) {
                input->zero_range_yards = atof(v);
            } else if (strcmp(k, "sight_height") == 0) {
                input->sight_height_inches = atof(v);
            }
        }
    }

    fclose(fp);
    return 0;
}

const char* drag_model_name(DragModel model) {
    return (model == DRAG_G7) ? "G7" : "G1";
}

void write_output(FILE *fp, ProjectileInput *input) {
    fprintf(fp, "=== Ballistic Calculator ===\n");
    fprintf(fp, "Load: %s\n\n", input->name);
    fprintf(fp, "Input Parameters:\n");
    fprintf(fp, "  Muzzle Velocity:  %.1f fps\n", input->muzzle_velocity_fps);
    fprintf(fp, "  Bullet Weight:    %.1f gr\n", input->bullet_weight_gr);
    fprintf(fp, "  Ballistic Coef:   %.3f (%s)\n", input->ballistic_coef, drag_model_name(input->drag_model));
    fprintf(fp, "  Zero Range:       %.0f yards\n", input->zero_range_yards);
    fprintf(fp, "  Sight Height:     %.2f inches\n", input->sight_height_inches);
    fprintf(fp, "\n");
    
    // Calculate trajectory with drag
    TrajectoryPoint points[32];
    int num_points;
    calc_drag_trajectory(input, points, &num_points, 32);
    
    // Print trajectory table
    fprintf(fp, "Trajectory Table (%s Drag Model):\n", drag_model_name(input->drag_model));
    fprintf(fp, "-----------------------------------------------------------------\n");
    fprintf(fp, " Range     Drop     Velocity      Energy       Time\n");
    fprintf(fp, " (yards)   (inches) (fps)         (ft-lbs)     (sec)\n");
    fprintf(fp, "-----------------------------------------------------------------\n");
    
    for (int i = 0; i < num_points; i++) {
        TrajectoryPoint *p = &points[i];
        fprintf(fp, " %5.0f     %+7.1f  %7.1f       %7.1f      %.3f\n",
                p->range_yards,
                p->drop_inches,
                p->velocity_fps,
                p->energy_ftlbs,
                p->time_sec);
    }
    fprintf(fp, "-----------------------------------------------------------------\n");
    
    // Show muzzle energy and remaining energy at max range
    if (num_points > 0) {
        double muzzle_energy = points[0].energy_ftlbs;
        double final_energy = points[num_points-1].energy_ftlbs;
        double final_velocity = points[num_points-1].velocity_fps;
        fprintf(fp, "\nMuzzle Energy: %.1f ft-lbs\n", muzzle_energy);
        fprintf(fp, "Energy at %d yards: %.1f ft-lbs (%.1f%% retained)\n",
                (int)points[num_points-1].range_yards,
                final_energy, 100.0 * final_energy / muzzle_energy);
        fprintf(fp, "Velocity at %d yards: %.1f fps (%.1f%% retained)\n",
                (int)points[num_points-1].range_yards,
                final_velocity, 100.0 * final_velocity / input->muzzle_velocity_fps);
    }
}

void print_usage(const char *prog_name) {
    printf("Usage: %s <input_file> [output_file]\n", prog_name);
    printf("\n");
    printf("If output_file is not specified, output goes to stdout.\n");
    printf("\n");
    printf("Input file format:\n");
    printf("  name=.308 Win 168gr SMK\n");
    printf("  muzzle_velocity=2650\n");
    printf("  bullet_weight=168\n");
    printf("  ballistic_coef=0.462\n");
    printf("  zero_range=100\n");
    printf("  sight_height=1.5\n");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    // Parse input file
    ProjectileInput input;
    if (parse_input_file(argv[1], &input) != 0) {
        return 1;
    }

    // Determine output destination
    FILE *output = stdout;
    if (argc >= 3) {
        output = fopen(argv[2], "w");
        if (!output) {
            fprintf(stderr, "Error: Cannot open output file '%s'\n", argv[2]);
            return 1;
        }
    }

    // Generate output
    write_output(output, &input);

    // Cleanup
    if (output != stdout) {
        fclose(output);
        printf("Output written to: %s\n", argv[2]);
    }

    return 0;
}