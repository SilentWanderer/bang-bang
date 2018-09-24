library("shape")
library("scales")
library("png")

toDegrees <- function(rad) {(rad * 180) / (pi)}
toRadians <- function(deg) {(deg * pi) / (180)}

fieldWidth <- 396
fieldHeight <- 27 * 12

fieldImage <- readPNG("/home/stephen/code/drivecontrol/field.png")
lim <- par()

tracking <- read.csv("/home/stephen/code/drivecontrol/tracking.csv")
trajectory <- read.csv("/home/stephen/code/drivecontrol/trajectory.csv")
  
plotField <- function() {
  plot( c(0), c(0), type = "l", xlim = c(0, fieldWidth), ylim = c(0, fieldHeight), xlab = "X (inches)", ylab = "Y (inches)")
  rasterImage(fieldImage, lim$usr[1], lim$usr[3], lim$usr[2], lim$usr[4])
}

drawtracking <- function(tracking_x, tracking_y) {
  lines( tracking_x, tracking_y, col = "red")
}

drawtrajectory <- function(trajectory_x, trajectory_y) {
  lines(trajectory_x, trajectory_y, col = "green")
}

drawRobot <- function(width, height, x, y, heading) {
  filledrectangle(wx = width, wy = height, col = alpha(col = "purple", 0.5), 
                  mid = c(x, y), angle = toDegrees(heading))

}

animate <- function(tracking_x, tracking_y, tracking_heading, trajectory_x, trajectory_y, dt) {
  library("animation")

  for(i in 1:length(trajectory_x)) {
    plotField()
    
    drawtrajectory(trajectory_x, trajectory_y)
    drawtracking(tracking_x[1:i], tracking_y[1:i])
    drawRobot(38.66, 33.91, tracking_x[i], tracking_y[i], toRadians(tracking_heading[i]))
  }

}

saveVideo({
  
  animate(tracking[, 1], tracking[, 2], tracking[, 3], trajectory[, 1], trajectory[, 2], 0.01)
  
}, interval = dt, ani.width = lim$usr[2] - lim$usr[1], ani.height = lim$usr[4] - lim$usr[3], video.name = "test.mp4")
