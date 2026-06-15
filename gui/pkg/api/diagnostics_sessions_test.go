package api

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func newSessionsRouter(db types.IDBProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	g := r.Group("/diagnostics")
	g.GET("/sessions", getSessions(db))
	g.DELETE("/sessions", deleteSessions(db))
	return r
}

func seedSessions(t *testing.T, db types.IDBProvider, n int) {
	t.Helper()
	sessions := make([]MowingSession, n)
	for i := range sessions {
		sessions[i] = MowingSession{ID: string(rune('a' + i)), Status: "completed"}
	}
	data, err := json.Marshal(sessions)
	require.NoError(t, err)
	require.NoError(t, db.Set(sessionsDBKey, data))
}

func TestDeleteSessions_ClearsHistory(t *testing.T) {
	db := types.NewMockDBProvider()
	seedSessions(t, db, 3)
	r := newSessionsRouter(db)

	// Confirm 3 sessions are present.
	rec := httptest.NewRecorder()
	r.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/diagnostics/sessions", nil))
	var before MowingSessionList
	require.NoError(t, json.Unmarshal(rec.Body.Bytes(), &before))
	assert.Equal(t, 3, before.Total)

	// Clear.
	del := httptest.NewRecorder()
	r.ServeHTTP(del, httptest.NewRequest(http.MethodDelete, "/diagnostics/sessions", nil))
	assert.Equal(t, http.StatusOK, del.Code)

	// History is now empty.
	rec2 := httptest.NewRecorder()
	r.ServeHTTP(rec2, httptest.NewRequest(http.MethodGet, "/diagnostics/sessions", nil))
	var after MowingSessionList
	require.NoError(t, json.Unmarshal(rec2.Body.Bytes(), &after))
	assert.Equal(t, 0, after.Total)
}

func TestDeleteSessions_IdempotentOnEmpty(t *testing.T) {
	db := types.NewMockDBProvider()
	r := newSessionsRouter(db)

	del := httptest.NewRecorder()
	r.ServeHTTP(del, httptest.NewRequest(http.MethodDelete, "/diagnostics/sessions", nil))
	assert.Equal(t, http.StatusOK, del.Code)
}
