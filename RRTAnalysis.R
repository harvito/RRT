if(!require(data.table)){install.packages("data.table")}
if(!require(ggplot2)){install.packages("ggplot2")}

#read csv
RRT1Data <- read.csv("RRT1Data.csv")
RRT2Data <- read.csv("RRT2Data.csv")

#factorize
RRT1Data$step <= as.factor(RRT1Data$step)
RRT2Data$robLen <= as.factor(RRT2Data$robLen)

# make plots and save them
ggplot(RRT1Data, aes(x=step, y=numIt)) + geom_boxplot() + scale_y_continuous(trans="log10") + ggtitle("Number of Iterations of RRT Loop vs. Step Size") + xlab("Step Size (Pixels)") + ylab("Number of Iterations")
ggsave("numItvsStepSize.pdf", device="pdf", width=6.5, height=4)

ggplot(RRT1Data, aes(x=step, y=pathLen)) + geom_boxplot() + ggtitle("Length of RRT Path vs. Step Size") + xlab("Step Size (Pixels)") + ylab("Path Length (Pixels)")
ggsave("pathLenvsStepSize.pdf", device="pdf", width=6.5, height=4)

ggplot(RRT2Data, aes(x=robLen, y=numIt)) + geom_boxplot() + ggtitle("Number of Iterations of RRT Loop vs. Robot Length") + xlab("Robot Length (Pixels)") + ylab("Number of Iterations")
ggsave("numItvsRobotLen.pdf", device="pdf", width=6.5, height=4)
