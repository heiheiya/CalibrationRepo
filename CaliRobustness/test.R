direction<-0:2
fnum<-0:30

deltay<-function(input)
{
  tmp<-input[,1]
  
  return(mean(tmp))
}

readData <- function(phoneFiles)
{
  data <- lapply(phoneFiles, read.table)
}

opar<-par(mfrow=c(3,1), mar=c(2,2,2,2), oma=c(1,1,1,1))
titles<-c('DeltaY vs X Error', 'DeltaY vs Y Error', 'DeltaY vs Z Error')
for (d in 0:2)
{
  getfilename <- function(phoneID)
  {
    paste(
      'resource/phone',phoneID,'/1/result/T/',d,'/result_T_',d,'_',fnum,".txt",sep =
        ""
    )
  }
  
  phoneID <- c(2,5,13)
  
  filenames <- lapply(phoneID, getfilename)
  
 
  
  alldata_x <- lapply(filenames, readData)
  
  deltays_x <- sapply(alldata_x[[1]],deltay)
  plot(
    1:31, rep(0,31), ylim = c(0,1.3), type = 'b', main = titles[d], ylab =
      'DeltaY'
  )
  
  color <- c('red','green','blue')
  for (i in c(1:3))
  {
    deltays_x <- sapply(alldata_x[[i]],deltay)
    deltays_x <-
      (deltays_x - min(deltays_x)) / (max(deltays_x) - min(deltays_x))
    points(
      1:31, deltays_x, col = color[i], type = 'b', ylim = c(0,1.3)
    )
  }
  legend("topright", c("2", "5", "13"), col = color, lty = 1)
}

par(opar)

#opar<-par(mfrow=c(3,1))






filenames<-paste('resource/phone5/1/result/T/',1,'/result_T_',1,'_',fnum,".txt",sep="")

alldata_y<-lapply(filenames, read.table)

deltays_y<-lapply(alldata_y,deltay)

plot(1:31, deltays_y, col=1, type='b', main='DeltaY vs Y Error',ylab='DeltaY')



filenames<-paste('resource/phone5/1/result/T/',2,'/result_T_',2,'_',fnum,".txt",sep="")

alldata_z<-lapply(filenames, read.table)
deltays_z<-lapply(alldata_z,deltay)

plot(1:31, deltays_z, col=1, type='b', main='DeltaY vs Z Error', ylab='DeltaY')

par(opar)


