# Example Data
library(plyr)
library("readxl")
library(tidyverse)
library(lubridate)
library(ggplot2)
library(dplyr)
library(multcomp)
library(car)
library(ggthemes)
library(agricolae)
library(foreign)

library(gtable)
library(grid)
grid.newpage()

my_data <- read_excel("energy.xlsx")
my_data
#Normalized Data
normalized = (x-min(x))/(max(x)-min(x))

p <- ggplot(my_data, aes(x = time))
p <- p + geom_line(aes(y = mWh_energy, colour = "Energy Consumption"), size=1.5)


#p <- p + geom_line(aes(y = mWh_bytes, colour = "Energy"))


#p <- p + scale_y_continuous(sec.axis = sec_axis(~.*1, name = "Energy mWh"))


p <- p + scale_colour_manual(values = c("blue", "red"))
p <- p + labs(y = "Energy Consumption [mWh])",
              x = "Processing Time [minutes]",
              colour = "")
p <- p + theme(legend.position = "top")
p <- p + theme(
  axis.text = element_text(size = 24),
  legend.spacing.x = unit(0.75, 'cm'),
  legend.text = element_text(size = 24),
  # Change legend key size and key width
  legend.key.size = unit(0.75, "cm"),
  legend.key.width = unit(1.40,"cm"))
  
 
p <- p + theme(axis.title.x = element_text(size=24))
# y axis title
p <- p + theme(axis.title.y = element_text(size=24))
p
#c(0.8, 0.9)

