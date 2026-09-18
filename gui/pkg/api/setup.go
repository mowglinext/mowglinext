package api

import (
	"bufio"
	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"io"
)

func SetupRoutes(r *gin.RouterGroup, provider types.IFirmwareProvider) {
	group := r.Group("/setup")
	FlashBoard(group, provider)
	// USB DFU is optional until the appliance wires its physical USB/runtime
	// dependencies.  This preserves the established ST-Link endpoint.
	if updater, ok := provider.(types.IFirmwareUSBUpdater); ok {
		FirmwareUSBUpdateRoutes(group, updater)
	}
}

func FirmwareUSBUpdateRoutes(r *gin.RouterGroup, updater types.IFirmwareUSBUpdater) {
	r.POST("/firmware-update", func(c *gin.Context) {
		var request types.FirmwareUpdateRequest
		if err := c.ShouldBindJSON(&request); err != nil {
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}
		snapshot, attached, err := updater.StartFirmwareUSBUpdate(c.Request.Context(), request)
		if err != nil {
			c.JSON(409, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(202, gin.H{"operation": snapshot, "attached": attached})
	})
	r.GET("/firmware-update/:id", func(c *gin.Context) {
		snapshot, ok := updater.FirmwareUSBUpdateSnapshot(c.Param("id"))
		if !ok {
			c.JSON(404, ErrorResponse{Error: "firmware update not found"})
			return
		}
		c.JSON(200, snapshot)
	})
	r.POST("/firmware-update/:id/cancel", func(c *gin.Context) {
		snapshot, err := updater.CancelFirmwareUSBUpdate(c.Param("id"))
		if err != nil {
			c.JSON(409, gin.H{"operation": snapshot, "error": err.Error()})
			return
		}
		c.JSON(200, snapshot)
	})
}

// FlashBoard flash the mower board with the given config
//
// @Summary flash the mower board with the given config
// @Description flash the mower board with the given config
// @Tags setup
// @Accept  json
// @Produce  text/event-stream
// @Param settings body types.FirmwareConfig true "config"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /setup/flashBoard [post]
func FlashBoard(r *gin.RouterGroup, provider types.IFirmwareProvider) gin.IRoutes {
	return r.POST("/flashBoard", func(c *gin.Context) {
		var config types.FirmwareConfig
		var err error
		err = c.BindJSON(&config)
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		reader, writer := io.Pipe()
		rd := bufio.NewReader(reader)
		go func() {
			err = provider.FlashFirmware(writer, config)
			if err != nil {
				writer.CloseWithError(err)
			} else {
				writer.Close()
			}
		}()
		c.Stream(func(w io.Writer) bool {
			line, _, err2 := rd.ReadLine()
			if err2 != nil {
				if err2 == io.EOF {
					c.SSEvent("end", "end")
					return false
				}
				c.SSEvent("error", err2.Error())
				return false
			}
			c.SSEvent("message", string(line))
			return true
		})
	})
}
