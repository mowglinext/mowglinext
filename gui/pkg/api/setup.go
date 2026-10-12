package api

import (
	"bufio"
	"io"
	"strings"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

func SetupRoutes(r *gin.RouterGroup, provider types.IFirmwareProvider) {
	group := r.Group("/setup")
	FlashBoard(group, provider)
	AvailableFirmware(group, provider)
}

// AvailableFirmware returns the prebuilt firmware a flash would install
//
// @Summary prebuilt firmware available for the saved board
// @Description Version and protocol of the prebuilt firmware that /setup/flashBoard would install for the saved board selection, taken from this installation's release (or the latest stable one when it carries none).
// @Tags setup
// @Produce  json
// @Success 200 {object} types.FirmwareAvailability
// @Failure 502 {object} ErrorResponse
// @Router /setup/firmware/available [get]
func AvailableFirmware(r *gin.RouterGroup, provider types.IFirmwareProvider) gin.IRoutes {
	return r.GET("/firmware/available", func(c *gin.Context) {
		result, err := provider.AvailableFirmware()
		if err != nil {
			c.JSON(502, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, result)
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
			event, payload := flashStreamEvent(string(line))
			c.SSEvent(event, payload)
			return true
		})
	})
}

// flashStreamEvent classifies one line of the flash log for the SSE stream: a
// line the provider marked as a stage transition becomes a `stage` event
// carrying the JSON FlashStageEvent, every other line is a plain log `message`.
func flashStreamEvent(line string) (event, payload string) {
	if strings.HasPrefix(line, types.FlashStageMarker) {
		return "stage", strings.TrimPrefix(line, types.FlashStageMarker)
	}
	return "message", line
}
