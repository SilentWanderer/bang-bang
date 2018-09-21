library("shape")
library("scales")

toDegrees <- function(rad) {(rad * 180) / (pi)}
toRadians <- function(deg) {(deg * pi) / (180)}

  
drawtracking <- function(tracking_x, tracking_y) {
  matlines( tracking_x, tracking_y, col = "red")
}

drawtrajectory <- function(trajectory_x, trajectory_y) {
  matlines(trajectory_x, trajectory_y, col = "green")
}

drawRobot <- function(width, height, x, y, heading) {
  filledrectangle(wx = width, wy = height, col = alpha(col = "purple", 0.5), 
                  mid = c(x, y), angle = toDegrees(heading))

}

animate <- function(tracking_x, tracking_y, tracking_heading, trajectory_x, trajectory_y, dt) {
  library("animation")
  saveVideo({
    for(i in 1:length(trajectory_x)) {
      plot(x=c(),y=c(),xlim = c(-60, 54 * 12), ylim = c(-60, 27 * 12))
      drawtrajectory(trajectory_x, trajectory_y)
      drawtracking(tracking_x[1:i], tracking_y[1:i])
      drawRobot(38.66, 33.91, tracking_x[i], tracking_y[i], toRadians(tracking_heading[i]))
    }
  }, interval = dt, ani.width = 1920, ani.height = 1080, video.name = "test.mp4")

}

tracking <- read.csv("/home/stephen/code/drivecontrol/tracking.csv")
trajectory <- read.csv("/home/stephen/code/drivecontrol/tracking.csv")

matplot( c(0), c(0), type = "line", xlim = c(0, 54 * 12), ylim = c(0, 27 * 12), xlab = "X (inches)", ylab = "Y (inches)")

animate(tracking[, 1], tracking[, 2], tracking[, 3], trajectory[, 1], trajectory[, 2], 0.01)
